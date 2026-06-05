#include "fg_runtime.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <variant>

#include <AL/al.h>
#include <AL/alc.h>
#include <tinyxml2.h>

#define DR_WAV_IMPLEMENTATION
#include <dr_wav.h>

namespace flight {
namespace {

constexpr double kMetersPerSecondToKnots = 1.9438444924406;
constexpr double kMetersToFeet = 3.2808398950131;
constexpr double kMaxTransitStopSeconds = 0.1;

std::string Trim(std::string text) {
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) { return std::isspace(c) != 0; });
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string NormalizePropertyPath(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    while (!path.empty() && path.front() == '/') {
        path.erase(path.begin());
    }
    while (!path.empty() && path.back() == '/') {
        path.pop_back();
    }
    return path;
}

std::string ElementText(const tinyxml2::XMLElement* element, const char* childName, const char* fallback = "") {
    const tinyxml2::XMLElement* child = element ? element->FirstChildElement(childName) : nullptr;
    const char* text = child ? child->GetText() : nullptr;
    return text ? Trim(text) : std::string(fallback);
}

double ElementDouble(const tinyxml2::XMLElement* element, const char* childName, double fallback = 0.0) {
    const std::string text = ElementText(element, childName);
    if (text.empty()) {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    return end != text.c_str() ? value : fallback;
}

bool TextToBool(const std::string& text) {
    std::string lower = text;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower == "1" || lower == "true" || lower == "yes" || lower == "y" || lower == "on";
}

struct PathPart {
    std::string name;
    size_t index = 0;
};

std::vector<PathPart> ParsePropertyPath(const std::string& rawPath) {
    std::vector<PathPart> parts;
    std::string path = NormalizePropertyPath(rawPath);
    size_t cursor = 0;
    while (cursor < path.size()) {
        const size_t slash = path.find('/', cursor);
        std::string segment = path.substr(cursor, slash == std::string::npos ? std::string::npos : slash - cursor);
        cursor = slash == std::string::npos ? path.size() : slash + 1;
        if (segment.empty()) {
            continue;
        }
        PathPart part{};
        const size_t open = segment.find('[');
        const size_t close = segment.find(']', open == std::string::npos ? 0 : open);
        if (open != std::string::npos && close != std::string::npos && close > open + 1) {
            part.name = segment.substr(0, open);
            part.index = static_cast<size_t>(std::max(0, std::atoi(segment.substr(open + 1, close - open - 1).c_str())));
        } else {
            part.name = segment;
        }
        parts.push_back(std::move(part));
    }
    return parts;
}

class FgPropertyTree {
public:
    using Value = std::variant<std::monostate, bool, int, double, std::string>;

    struct Node {
        std::string name;
        size_t index = 0;
        Node* parent = nullptr;
        Value value;
        std::string aliasPath;
        std::map<std::string, std::vector<std::unique_ptr<Node>>> children;
    };

    FgPropertyTree() {
        m_root.name = "";
    }

    Node* GetNode(const std::string& path, bool create) {
        Node* current = &m_root;
        for (const PathPart& part : ParsePropertyPath(path)) {
            if (part.name.empty()) {
                continue;
            }
            auto& siblings = current->children[part.name];
            if (siblings.size() <= part.index) {
                if (!create) {
                    return nullptr;
                }
                while (siblings.size() <= part.index) {
                    auto child = std::make_unique<Node>();
                    child->name = part.name;
                    child->index = siblings.size();
                    child->parent = current;
                    siblings.push_back(std::move(child));
                }
            }
            current = siblings[part.index].get();
        }
        return ResolveAlias(current, 0);
    }

    const Node* GetNode(const std::string& path) const {
        return const_cast<FgPropertyTree*>(this)->GetNode(path, false);
    }

    void SetAlias(const std::string& path, const std::string& targetPath) {
        if (Node* node = GetNode(path, true)) {
            node->aliasPath = NormalizePropertyPath(targetPath);
        }
    }

    double GetDouble(const std::string& path, double fallback = 0.0) const {
        const Node* node = GetNode(path);
        return node ? ValueAsDouble(node->value, fallback) : fallback;
    }

    bool GetBool(const std::string& path, bool fallback = false) const {
        const Node* node = GetNode(path);
        return node ? ValueAsBool(node->value, fallback) : fallback;
    }

    std::string GetString(const std::string& path, const std::string& fallback = "") const {
        const Node* node = GetNode(path);
        return node ? ValueAsString(node->value, fallback) : fallback;
    }

    void SetDouble(const std::string& path, double value) {
        if (Node* node = GetNode(path, true)) {
            node->value = value;
        }
    }

    void SetBool(const std::string& path, bool value) {
        if (Node* node = GetNode(path, true)) {
            node->value = value;
        }
    }

    void SetString(const std::string& path, std::string value) {
        if (Node* node = GetNode(path, true)) {
            node->value = std::move(value);
        }
    }

    void SetFromText(const std::string& path, const std::string& text, const std::string& typeHint) {
        Node* node = GetNode(path, true);
        if (!node) {
            return;
        }
        if (typeHint == "bool") {
            node->value = TextToBool(text);
        } else if (typeHint == "int" || typeHint == "long") {
            node->value = std::atoi(text.c_str());
        } else if (typeHint == "double" || typeHint == "float") {
            node->value = std::strtod(text.c_str(), nullptr);
        } else if (typeHint == "string") {
            node->value = text;
        } else {
            char* end = nullptr;
            const double numeric = std::strtod(text.c_str(), &end);
            if (end != text.c_str() && *end == '\0') {
                node->value = numeric;
            } else {
                node->value = text;
            }
        }
    }

    std::vector<FgPropertyDebugInfo> Find(const std::string& query, size_t limit) const {
        std::vector<FgPropertyDebugInfo> out;
        const std::string normalizedQuery = NormalizePropertyPath(query);
        Walk(&m_root, "", [&](const Node& node, const std::string& path) {
            if (out.size() >= limit || path.empty()) {
                return;
            }
            if (normalizedQuery.empty() || path.find(normalizedQuery) != std::string::npos) {
                out.push_back({"/" + path, ValueAsString(node.value, "")});
            }
        });
        return out;
    }

private:
    Node m_root;

    Node* ResolveAlias(Node* node, int depth) {
        if (!node || node->aliasPath.empty() || depth > 16) {
            return node;
        }
        Node* target = GetNode(node->aliasPath, true);
        return target == node ? node : ResolveAlias(target, depth + 1);
    }

    static double ValueAsDouble(const Value& value, double fallback) {
        if (const auto* v = std::get_if<double>(&value)) {
            return *v;
        }
        if (const auto* v = std::get_if<int>(&value)) {
            return static_cast<double>(*v);
        }
        if (const auto* v = std::get_if<bool>(&value)) {
            return *v ? 1.0 : 0.0;
        }
        if (const auto* v = std::get_if<std::string>(&value)) {
            char* end = nullptr;
            const double parsed = std::strtod(v->c_str(), &end);
            return end != v->c_str() ? parsed : fallback;
        }
        return fallback;
    }

    static bool ValueAsBool(const Value& value, bool fallback) {
        if (const auto* v = std::get_if<bool>(&value)) {
            return *v;
        }
        if (const auto* v = std::get_if<double>(&value)) {
            return std::abs(*v) > 1e-9;
        }
        if (const auto* v = std::get_if<int>(&value)) {
            return *v != 0;
        }
        if (const auto* v = std::get_if<std::string>(&value)) {
            return TextToBool(*v);
        }
        return fallback;
    }

    static std::string ValueAsString(const Value& value, const std::string& fallback) {
        if (const auto* v = std::get_if<std::string>(&value)) {
            return *v;
        }
        if (const auto* v = std::get_if<double>(&value)) {
            std::ostringstream stream;
            stream << *v;
            return stream.str();
        }
        if (const auto* v = std::get_if<int>(&value)) {
            return std::to_string(*v);
        }
        if (const auto* v = std::get_if<bool>(&value)) {
            return *v ? "true" : "false";
        }
        return fallback;
    }

    template <typename Fn>
    static void Walk(const Node* node, const std::string& path, Fn&& fn) {
        fn(*node, path);
        for (const auto& [name, siblings] : node->children) {
            for (const auto& child : siblings) {
                std::string childPath = path.empty() ? child->name : path + "/" + child->name;
                if (child->index > 0) {
                    childPath += "[" + std::to_string(child->index) + "]";
                }
                Walk(child.get(), childPath, fn);
            }
        }
    }
};

class Expression {
public:
    virtual ~Expression() = default;
    virtual double Eval(const FgPropertyTree& tree) const = 0;
};

using ExpressionPtr = std::unique_ptr<Expression>;

class ConstExpression final : public Expression {
public:
    explicit ConstExpression(double value) : m_value(value) {}
    double Eval(const FgPropertyTree&) const override { return m_value; }
private:
    double m_value = 0.0;
};

class PropertyExpression final : public Expression {
public:
    explicit PropertyExpression(std::string path) : m_path(std::move(path)) {}
    double Eval(const FgPropertyTree& tree) const override { return tree.GetDouble(m_path); }
private:
    std::string m_path;
};

class ListExpression final : public Expression {
public:
    enum class Op { Sum, Product, Min, Max };
    explicit ListExpression(Op op) : m_op(op) {}
    void Add(ExpressionPtr expression) { m_children.push_back(std::move(expression)); }
    double Eval(const FgPropertyTree& tree) const override {
        if (m_op == Op::Product) {
            double value = 1.0;
            for (const auto& child : m_children) {
                value *= child->Eval(tree);
            }
            return value;
        }
        if (m_op == Op::Min || m_op == Op::Max) {
            if (m_children.empty()) {
                return 0.0;
            }
            double value = m_children.front()->Eval(tree);
            for (size_t i = 1; i < m_children.size(); ++i) {
                value = (m_op == Op::Min) ? std::min(value, m_children[i]->Eval(tree)) : std::max(value, m_children[i]->Eval(tree));
            }
            return value;
        }
        double value = 0.0;
        for (const auto& child : m_children) {
            value += child->Eval(tree);
        }
        return value;
    }
private:
    Op m_op = Op::Sum;
    std::vector<ExpressionPtr> m_children;
};

class TableExpression final : public Expression {
public:
    explicit TableExpression(ExpressionPtr input) : m_input(std::move(input)) {}
    void AddEntry(double ind, double dep) { m_entries[ind] = dep; }
    double Eval(const FgPropertyTree& tree) const override {
        if (m_entries.empty()) {
            return 0.0;
        }
        const double x = m_input ? m_input->Eval(tree) : 0.0;
        auto upper = m_entries.upper_bound(x);
        if (upper == m_entries.end()) {
            return m_entries.rbegin()->second;
        }
        if (upper == m_entries.begin()) {
            return upper->second;
        }
        auto lower = upper;
        --lower;
        const double t = (x - lower->first) / (upper->first - lower->first);
        return lower->second + (upper->second - lower->second) * t;
    }
private:
    ExpressionPtr m_input;
    std::map<double, double> m_entries;
};

ExpressionPtr ParseExpression(const tinyxml2::XMLElement* element) {
    if (!element) {
        return std::make_unique<ConstExpression>(0.0);
    }
    const std::string name = element->Name();
    if (name == "expression") {
        return ParseExpression(element->FirstChildElement());
    }
    if (name == "property") {
        return std::make_unique<PropertyExpression>(Trim(element->GetText() ? element->GetText() : ""));
    }
    if (name == "value") {
        return std::make_unique<ConstExpression>(std::strtod(Trim(element->GetText() ? element->GetText() : "0").c_str(), nullptr));
    }
    if (name == "sum" || name == "product" || name == "min" || name == "max") {
        auto op = ListExpression::Op::Sum;
        if (name == "product") {
            op = ListExpression::Op::Product;
        } else if (name == "min") {
            op = ListExpression::Op::Min;
        } else if (name == "max") {
            op = ListExpression::Op::Max;
        }
        auto list = std::make_unique<ListExpression>(op);
        for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
            list->Add(ParseExpression(child));
        }
        return list;
    }
    if (name == "table" || name == "interpolation") {
        ExpressionPtr input;
        for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
            if (std::string(child->Name()) != "entry") {
                input = ParseExpression(child);
                break;
            }
        }
        auto table = std::make_unique<TableExpression>(std::move(input));
        for (const tinyxml2::XMLElement* entry = element->FirstChildElement("entry"); entry; entry = entry->NextSiblingElement("entry")) {
            table->AddEntry(ElementDouble(entry, "ind"), ElementDouble(entry, "dep"));
        }
        return table;
    }
    return std::make_unique<ConstExpression>(ElementDouble(element, "value", 0.0));
}

class Condition {
public:
    virtual ~Condition() = default;
    virtual bool Test(const FgPropertyTree& tree) const = 0;
};

using ConditionPtr = std::unique_ptr<Condition>;

class BoolCondition final : public Condition {
public:
    explicit BoolCondition(bool value) : m_value(value) {}
    bool Test(const FgPropertyTree&) const override { return m_value; }
private:
    bool m_value = false;
};

class PropertyCondition final : public Condition {
public:
    explicit PropertyCondition(std::string path) : m_path(std::move(path)) {}
    bool Test(const FgPropertyTree& tree) const override { return tree.GetBool(m_path); }
private:
    std::string m_path;
};

class NotCondition final : public Condition {
public:
    explicit NotCondition(ConditionPtr child) : m_child(std::move(child)) {}
    bool Test(const FgPropertyTree& tree) const override { return !m_child || !m_child->Test(tree); }
private:
    ConditionPtr m_child;
};

class GroupCondition final : public Condition {
public:
    enum class Op { And, Or };
    explicit GroupCondition(Op op) : m_op(op) {}
    void Add(ConditionPtr child) { m_children.push_back(std::move(child)); }
    bool Test(const FgPropertyTree& tree) const override {
        if (m_op == Op::Or) {
            return std::any_of(m_children.begin(), m_children.end(), [&](const auto& child) { return child->Test(tree); });
        }
        return std::all_of(m_children.begin(), m_children.end(), [&](const auto& child) { return child->Test(tree); });
    }
private:
    Op m_op = Op::And;
    std::vector<ConditionPtr> m_children;
};

class CompareCondition final : public Condition {
public:
    enum class Op { Less, LessEqual, Greater, GreaterEqual, Equal, NotEqual };
    CompareCondition(Op op, ExpressionPtr left, ExpressionPtr right)
        : m_op(op), m_left(std::move(left)), m_right(std::move(right)) {}
    bool Test(const FgPropertyTree& tree) const override {
        const double l = m_left ? m_left->Eval(tree) : 0.0;
        const double r = m_right ? m_right->Eval(tree) : 0.0;
        switch (m_op) {
        case Op::Less: return l < r;
        case Op::LessEqual: return l <= r;
        case Op::Greater: return l > r;
        case Op::GreaterEqual: return l >= r;
        case Op::Equal: return std::abs(l - r) <= 1e-6;
        case Op::NotEqual: return std::abs(l - r) > 1e-6;
        }
        return false;
    }
private:
    Op m_op = Op::Equal;
    ExpressionPtr m_left;
    ExpressionPtr m_right;
};

ExpressionPtr ParseConditionOperand(const tinyxml2::XMLElement* element) {
    return ParseExpression(element);
}

ConditionPtr ParseConditionNode(const tinyxml2::XMLElement* element) {
    if (!element) {
        return std::make_unique<BoolCondition>(true);
    }
    const std::string name = element->Name();
    if (name == "property") {
        return std::make_unique<PropertyCondition>(Trim(element->GetText() ? element->GetText() : ""));
    }
    if (name == "true") {
        return std::make_unique<BoolCondition>(true);
    }
    if (name == "false") {
        return std::make_unique<BoolCondition>(false);
    }
    if (name == "not") {
        return std::make_unique<NotCondition>(ParseConditionNode(element->FirstChildElement()));
    }
    if (name == "and" || name == "condition") {
        auto group = std::make_unique<GroupCondition>(GroupCondition::Op::And);
        for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
            group->Add(ParseConditionNode(child));
        }
        return group;
    }
    if (name == "or") {
        auto group = std::make_unique<GroupCondition>(GroupCondition::Op::Or);
        for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
            group->Add(ParseConditionNode(child));
        }
        return group;
    }
    const std::unordered_map<std::string, CompareCondition::Op> compareOps{
        {"less-than", CompareCondition::Op::Less},
        {"less-than-equals", CompareCondition::Op::LessEqual},
        {"greater-than", CompareCondition::Op::Greater},
        {"greater-than-equals", CompareCondition::Op::GreaterEqual},
        {"equals", CompareCondition::Op::Equal},
        {"not-equals", CompareCondition::Op::NotEqual},
    };
    if (const auto it = compareOps.find(name); it != compareOps.end()) {
        const tinyxml2::XMLElement* left = element->FirstChildElement();
        const tinyxml2::XMLElement* right = left ? left->NextSiblingElement() : nullptr;
        return std::make_unique<CompareCondition>(it->second, ParseConditionOperand(left), ParseConditionOperand(right));
    }
    return std::make_unique<BoolCondition>(true);
}

class PropertyListLoader {
public:
    explicit PropertyListLoader(std::filesystem::path aircraftRoot) : m_aircraftRoot(std::move(aircraftRoot)) {}

    bool Load(const std::filesystem::path& path, FgPropertyTree& tree, std::string& error) {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
            error = "Could not read PropertyList XML: " + path.string();
            return false;
        }
        const tinyxml2::XMLElement* root = doc.RootElement();
        if (!root) {
            error = "PropertyList XML has no root: " + path.string();
            return false;
        }
        LoadElement(root, "", path.parent_path(), tree, "");
        return true;
    }

    std::filesystem::path ResolvePath(const std::string& rawPath, const std::filesystem::path& baseDir) const {
        std::string path = rawPath;
        std::replace(path.begin(), path.end(), '\\', '/');
        while (!path.empty() && path.front() == '/') {
            path.erase(path.begin());
        }
        constexpr const char* prefix = "Aircraft/737-800YV/";
        if (path.rfind(prefix, 0) == 0) {
            return m_aircraftRoot / path.substr(std::char_traits<char>::length(prefix));
        }
        std::filesystem::path fsPath(path);
        if (fsPath.is_absolute()) {
            return fsPath;
        }
        std::filesystem::path baseResolved = baseDir / fsPath;
        if (std::filesystem::exists(baseResolved)) {
            return baseResolved;
        }
        return m_aircraftRoot / fsPath;
    }

private:
    std::filesystem::path m_aircraftRoot;

    static std::string ChildNameForPath(const tinyxml2::XMLElement* child, std::map<std::string, int>& counters) {
        std::string childName = child->Name();
        if (const char* n = child->Attribute("n")) {
            childName += "[" + std::string(n) + "]";
            counters[child->Name()] = std::max(counters[child->Name()], std::atoi(n) + 1);
        } else {
            const int index = counters[childName]++;
            if (index > 0) {
                childName += "[" + std::to_string(index) + "]";
            }
        }
        return childName;
    }

    void LoadElement(
        const tinyxml2::XMLElement* element,
        const std::string& parentPath,
        const std::filesystem::path& baseDir,
        FgPropertyTree& tree,
        const std::string& pathName) {
        if (std::string(element->Name()) == "PropertyList") {
            if (const char* include = element->Attribute("include")) {
                std::string ignored;
                Load(ResolvePath(include, baseDir), tree, ignored);
            }
            std::map<std::string, int> counters;
            for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
                LoadElement(child, parentPath, baseDir, tree, ChildNameForPath(child, counters));
            }
            return;
        }

        std::string name = pathName.empty() ? element->Name() : pathName;
        const std::string path = parentPath.empty() ? name : parentPath + "/" + name;
        if (const char* alias = element->Attribute("alias")) {
            tree.SetAlias(path, alias);
        }
        const char* type = element->Attribute("type");
        const char* text = element->GetText();
        if (text != nullptr) {
            const std::string value = Trim(text);
            if (!value.empty()) {
                tree.SetFromText(path, value, type ? type : "");
            }
        }
        std::map<std::string, int> counters;
        for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
            LoadElement(child, path, baseDir, tree, ChildNameForPath(child, counters));
        }
    }
};

struct SoundParam {
    ExpressionPtr expression;
    std::string property;
    enum class Internal { None, DtPlay, DtStop } internal = Internal::None;
    std::function<double(double)> fn;
    double factor = 1.0;
    double offset = 0.0;
    double min = 0.0;
    double max = 0.0;
    bool subtract = false;
};

struct SystemFilter {
    ExpressionPtr input;
    std::string outputProperty;
    double gain = 1.0;

    void Update(FgPropertyTree& tree) const {
        if (!outputProperty.empty() && input) {
            tree.SetDouble(outputProperty, input->Eval(tree) * gain);
        }
    }
};

struct SoundAxisValue {
    double constant = 0.0;
    std::string property;

    [[nodiscard]] double Eval(const FgPropertyTree& tree) const {
        return property.empty() ? constant : tree.GetDouble(property, constant);
    }
};

SoundAxisValue ParseSoundAxisValue(const tinyxml2::XMLElement* parent, const char* shortName, const char* meterName) {
    SoundAxisValue value{};
    const tinyxml2::XMLElement* child = parent ? parent->FirstChildElement(shortName) : nullptr;
    if (!child && meterName) {
        child = parent ? parent->FirstChildElement(meterName) : nullptr;
    }
    if (!child) {
        return value;
    }
    if (const tinyxml2::XMLElement* property = child->FirstChildElement("property")) {
        value.property = Trim(property->GetText() ? property->GetText() : "");
    }
    const char* text = child->GetText();
    if (text) {
        const std::string trimmed = Trim(text);
        if (!trimmed.empty()) {
            char* end = nullptr;
            const double parsed = std::strtod(trimmed.c_str(), &end);
            if (end != trimmed.c_str()) {
                value.constant = parsed;
            }
        }
    }
    return value;
}

struct SoundPosition {
    SoundAxisValue x;
    SoundAxisValue y;
    SoundAxisValue z;

    [[nodiscard]] bool Dynamic() const {
        return !x.property.empty() || !y.property.empty() || !z.property.empty();
    }

    void Apply(ALuint source, const FgPropertyTree& tree) const {
        alSource3f(
            source,
            AL_POSITION,
            static_cast<ALfloat>(x.Eval(tree)),
            static_cast<ALfloat>(y.Eval(tree)),
            static_cast<ALfloat>(z.Eval(tree)));
    }
};

struct SoundOrientation {
    SoundAxisValue x;
    SoundAxisValue y;
    SoundAxisValue z;
    float innerAngle = 360.0f;
    float outerAngle = 360.0f;
    float outerGain = 1.0f;
    bool enabled = false;

    void Apply(ALuint source, const FgPropertyTree& tree) const {
        if (!enabled) {
            return;
        }
        alSource3f(
            source,
            AL_DIRECTION,
            static_cast<ALfloat>(x.Eval(tree)),
            static_cast<ALfloat>(y.Eval(tree)),
            static_cast<ALfloat>(z.Eval(tree)));
        alSourcef(source, AL_CONE_INNER_ANGLE, innerAngle);
        alSourcef(source, AL_CONE_OUTER_ANGLE, outerAngle);
        alSourcef(source, AL_CONE_OUTER_GAIN, outerGain);
    }
};

SoundPosition ParseSoundPosition(const tinyxml2::XMLElement* position) {
    SoundPosition out{};
    out.x = ParseSoundAxisValue(position, "x", "x-m");
    out.y = ParseSoundAxisValue(position, "y", "y-m");
    out.z = ParseSoundAxisValue(position, "z", "z-m");
    return out;
}

SoundOrientation ParseSoundOrientation(const tinyxml2::XMLElement* orientation) {
    SoundOrientation out{};
    if (!orientation) {
        return out;
    }
    out.enabled = true;
    out.x = ParseSoundAxisValue(orientation, "x", "x-m");
    out.y = ParseSoundAxisValue(orientation, "y", "y-m");
    out.z = ParseSoundAxisValue(orientation, "z", "z-m");
    out.innerAngle = static_cast<float>(ElementDouble(orientation, "inner-angle", ElementDouble(orientation, "inner-angle-deg", 360.0)));
    out.outerAngle = static_cast<float>(ElementDouble(orientation, "outer-angle", ElementDouble(orientation, "outer-angle-deg", out.innerAngle)));
    out.outerGain = static_cast<float>(std::clamp(ElementDouble(orientation, "outer-gain", 1.0), 0.0, 1.0));
    return out;
}

std::string JoinStrings(const std::vector<std::string>& values, const char* separator) {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << separator;
        }
        oss << values[i];
    }
    return oss.str();
}

class AudioEngine {
public:
    ~AudioEngine() { Shutdown(); }

    bool Initialize(std::string& error) {
        m_device = alcOpenDevice(nullptr);
        if (!m_device) {
            error = "OpenAL could not open the default audio device";
            return false;
        }
        m_context = alcCreateContext(m_device, nullptr);
        if (!m_context || alcMakeContextCurrent(m_context) == ALC_FALSE) {
            error = "OpenAL could not create an audio context";
            Shutdown();
            return false;
        }
        alDistanceModel(AL_INVERSE_DISTANCE_CLAMPED);
        return true;
    }

    void Shutdown() {
        for (ALuint source : m_sources) {
            alDeleteSources(1, &source);
        }
        m_sources.clear();
        for (auto& [_, buffer] : m_buffers) {
            alDeleteBuffers(1, &buffer);
        }
        m_buffers.clear();
        if (m_context) {
            alcMakeContextCurrent(nullptr);
            alcDestroyContext(m_context);
            m_context = nullptr;
        }
        if (m_device) {
            alcCloseDevice(m_device);
            m_device = nullptr;
        }
    }

    std::optional<ALuint> LoadBuffer(const std::filesystem::path& path, std::string& error) {
        const std::string key = path.string();
        if (const auto it = m_buffers.find(key); it != m_buffers.end()) {
            return it->second;
        }

        unsigned int channels = 0;
        unsigned int sampleRate = 0;
        drwav_uint64 frameCount = 0;
        drwav_int16* samples = drwav_open_file_and_read_pcm_frames_s16(key.c_str(), &channels, &sampleRate, &frameCount, nullptr);
        if (!samples) {
            error = "dr_wav could not decode: " + key;
            return std::nullopt;
        }
        ALenum format = 0;
        if (channels == 1) {
            format = AL_FORMAT_MONO16;
        } else if (channels == 2) {
            format = AL_FORMAT_STEREO16;
        } else {
            drwav_free(samples, nullptr);
            error = "Unsupported WAV channel count " + std::to_string(channels) + " for " + key;
            return std::nullopt;
        }

        ALuint buffer = 0;
        alGenBuffers(1, &buffer);
        const auto byteCount = static_cast<ALsizei>(frameCount * channels * sizeof(drwav_int16));
        alBufferData(buffer, format, samples, byteCount, static_cast<ALsizei>(sampleRate));
        drwav_free(samples, nullptr);
        m_buffers.emplace(key, buffer);
        return buffer;
    }

    ALuint CreateSource(ALuint buffer, bool looped, float referenceDistance, float maxDistance, float x, float y, float z) {
        ALuint source = 0;
        alGenSources(1, &source);
        alSourcei(source, AL_BUFFER, static_cast<ALint>(buffer));
        alSourcei(source, AL_LOOPING, looped ? AL_TRUE : AL_FALSE);
        alSourcei(source, AL_SOURCE_RELATIVE, AL_TRUE);
        alSource3f(source, AL_POSITION, x, y, z);
        alSourcef(source, AL_REFERENCE_DISTANCE, std::max(0.1f, referenceDistance));
        alSourcef(source, AL_MAX_DISTANCE, std::max(referenceDistance, maxDistance));
        m_sources.push_back(source);
        return source;
    }

    void UpdateListener(float masterVolume) {
        alListener3f(AL_POSITION, 0.0f, 0.0f, 0.0f);
        alListener3f(AL_VELOCITY, 0.0f, 0.0f, 0.0f);
        const ALfloat orientation[] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f};
        alListenerfv(AL_ORIENTATION, orientation);
        alListenerf(AL_GAIN, masterVolume);
    }

private:
    ALCdevice* m_device = nullptr;
    ALCcontext* m_context = nullptr;
    std::unordered_map<std::string, ALuint> m_buffers;
    std::vector<ALuint> m_sources;
};

struct SoundEvent {
    enum class Mode { Once, Looped, InTransit };

    std::string name;
    std::string pathText;
    std::string category;
    std::string type;
    Mode mode = Mode::Once;
    std::string triggerProperty;
    ConditionPtr condition;
    std::vector<SoundParam> volumes;
    std::vector<SoundParam> pitches;
    SoundPosition position;
    SoundOrientation orientation;
    std::vector<std::string> unsupportedNodes;
    ALuint source = 0;
    bool active = false;
    bool loaded = false;
    bool lastCondition = false;
    double previousPropertyValue = 0.0;
    double dtPlay = 0.0;
    double dtStop = 0.0;
    double delay = 0.0;
    double stopping = 0.0;
    float lastVolume = 0.0f;
    float lastPitch = 1.0f;

    bool IsPlaying() const {
        if (source == 0) {
            return false;
        }
        ALint state = 0;
        alGetSourcei(source, AL_SOURCE_STATE, &state);
        return state == AL_PLAYING;
    }

    void Stop() {
        if (source != 0) {
            alSourceStop(source);
        }
        active = false;
        dtPlay = 0.0;
        stopping = 0.0;
    }

    static double EvalParam(const SoundParam& param, const FgPropertyTree& tree, double dtPlayValue, double dtStopValue) {
        double value = 1.0;
        if (param.expression) {
            value = param.expression->Eval(tree);
        } else if (!param.property.empty()) {
            value = tree.GetDouble(param.property);
        } else if (param.internal == SoundParam::Internal::DtPlay) {
            value = dtPlayValue;
        } else if (param.internal == SoundParam::Internal::DtStop) {
            value = dtStopValue;
        }
        if (param.fn) {
            value = param.fn(value);
        }
        value *= param.factor;
        if (param.max != 0.0 && value > param.max) {
            value = param.max;
        }
        if (value < param.min) {
            value = param.min;
        }
        return value;
    }

    double ComputeParams(const std::vector<SoundParam>& params, const FgPropertyTree& tree, bool pitch) const {
        double value = 1.0;
        double offset = 0.0;
        for (const SoundParam& param : params) {
            double v = EvalParam(param, tree, dtPlay, dtStop);
            if (param.subtract) {
                v += param.offset;
                if (v >= 0.0) {
                    value *= v;
                }
            } else if (v >= 0.0) {
                offset += param.offset;
                value *= v;
            }
        }
        double out = offset + value;
        if (!pitch) {
            out = std::clamp(out, 0.0, 1.0);
        } else {
            out = std::max(0.01, out);
        }
        return out;
    }

    double CategoryGain(const FgPropertyTree& tree) const {
        const double master = tree.GetDouble("/sim/sound/category/master", 1.0);
        if (type == "avionics") {
            return master * tree.GetDouble("/sim/sound/category/avionics", 1.0);
        }
        if (category == "engine" || category == "reverse" || category == "engineReverse") {
            return master * tree.GetDouble("/sim/sound/category/engine", 1.0);
        }
        if (category == "announcement" || category == "boarding" || category == "safety") {
            return master * tree.GetDouble("/sim/sound/category/announcement", 1.0);
        }
        if (category == "rain" || category == "wind" || category == "rumble") {
            return master * tree.GetDouble("/sim/sound/category/environment", 1.0);
        }
        if (category == "callout" || category == "gpws" || category == "wow" || category == "warning" || category == "altAlert") {
            return master * tree.GetDouble("/sim/sound/category/callout", 1.0);
        }
        return master * tree.GetDouble("/sim/sound/category/effects", 1.0);
    }

    void Update(FgPropertyTree& tree, double dt) {
        if (!loaded || source == 0) {
            return;
        }
        if (position.Dynamic()) {
            position.Apply(source, tree);
        }
        orientation.Apply(source, tree);

        bool nowCondition = false;
        if (condition) {
            nowCondition = condition->Test(tree);
        } else if (!triggerProperty.empty()) {
            if (mode == Mode::InTransit) {
                const double current = tree.GetDouble(triggerProperty);
                nowCondition = current != previousPropertyValue;
                previousPropertyValue = current;
            } else {
                nowCondition = tree.GetBool(triggerProperty);
            }
        }
        lastCondition = nowCondition;

        if (!nowCondition) {
            if (mode != Mode::InTransit || stopping > kMaxTransitStopSeconds) {
                if (IsPlaying()) {
                    alSourceStop(source);
                }
                active = false;
                dtStop += dt;
                dtPlay = 0.0;
            } else {
                stopping += dt;
            }
            return;
        }

        if (active && mode == Mode::Once) {
            if (IsPlaying()) {
                dtPlay += dt;
            } else {
                dtStop += dt;
                dtPlay = 0.0;
            }
        } else {
            dtPlay += dt;
            stopping = 0.0;
        }
        if (dtPlay < delay) {
            return;
        }
        if (!active) {
            alSourcei(source, AL_LOOPING, mode == Mode::Once ? AL_FALSE : AL_TRUE);
            alSourcePlay(source);
            active = true;
            dtStop = 0.0;
        }
        if (!IsPlaying() && mode != Mode::Once) {
            alSourcePlay(source);
        }
        if (IsPlaying()) {
            lastVolume = static_cast<float>(ComputeParams(volumes, tree, false) * CategoryGain(tree));
            lastVolume = std::clamp(lastVolume, 0.0f, 1.0f);
            lastPitch = static_cast<float>(ComputeParams(pitches, tree, true));
            alSourcef(source, AL_GAIN, lastVolume);
            alSourcef(source, AL_PITCH, lastPitch);
        }
    }
};

SoundParam ParseSoundParam(const tinyxml2::XMLElement* element, bool pitch, std::mt19937& rng) {
    SoundParam param{};
    param.offset = pitch ? 1.0 : 0.0;
    param.min = 0.0;
    param.max = 0.0;
    param.factor = 1.0;
    if (const tinyxml2::XMLElement* expression = element->FirstChildElement("expression")) {
        param.expression = ParseExpression(expression);
    }
    param.property = ElementText(element, "property");
    const std::string internal = ElementText(element, "internal");
    if (internal == "dt_play") {
        param.internal = SoundParam::Internal::DtPlay;
    } else if (internal == "dt_stop") {
        param.internal = SoundParam::Internal::DtStop;
    }
    param.factor = ElementDouble(element, "factor", 1.0);
    if (param.factor < 0.0) {
        param.factor = -param.factor;
        param.subtract = true;
    }
    param.offset = ElementDouble(element, "offset", param.offset);
    if (pitch) {
        const double randomRange = ElementDouble(element, "random", 0.0);
        if (randomRange > 0.0) {
            std::uniform_real_distribution<double> dist(0.0, randomRange);
            param.offset += dist(rng);
        }
    }
    param.min = ElementDouble(element, "min", 0.0);
    param.max = ElementDouble(element, "max", 0.0);
    const std::string type = ElementText(element, "type");
    if (type == "inv") {
        param.fn = [](double v) { return v == 0.0 ? 1e99 : 1.0 / v; };
    } else if (type == "abs") {
        param.fn = [](double v) { return std::abs(v); };
    } else if (type == "sqrt") {
        param.fn = [](double v) { return std::sqrt(std::abs(v)); };
    } else if (type == "log") {
        param.fn = [](double v) { return std::log10(std::abs(v) + 1e-9); };
    } else if (type == "ln") {
        param.fn = [](double v) { return std::log(std::abs(v) + 1e-9); };
    }
    return param;
}

} // namespace

struct FgRuntime::Impl {
    FgPropertyTree tree;
    std::filesystem::path aircraftRoot;
    std::vector<SystemFilter> filters;
    std::vector<SoundEvent> sounds;
    AudioEngine audio;
    std::mt19937 rng{0x737800u};
    std::string loadStatus;
    int unsupportedSoundNodeCount = 0;
    int missingSampleCount = 0;
    double elapsed = 0.0;

    bool Initialize(const std::filesystem::path& root, std::string& error) {
        aircraftRoot = root;
        if (!audio.Initialize(error)) {
            return false;
        }

        PropertyListLoader loader(aircraftRoot);
        const std::filesystem::path setXml = aircraftRoot / "737-800YV-set.xml";
        if (std::filesystem::exists(setXml)) {
            std::string ignored;
            loader.Load(setXml, tree, ignored);
        }

        SeedDefaults();

        if (!LoadFilters(loader, aircraftRoot / "Systems" / "fl2070_sound.xml", error)) {
            return false;
        }
        if (!LoadSounds(loader, aircraftRoot / "Sounds" / "737-sound.xml", error)) {
            return false;
        }

        loadStatus = "FlightGear audio: " + std::to_string(sounds.size()) + " sounds, " +
            std::to_string(filters.size()) + " filters";
        if (unsupportedSoundNodeCount > 0) {
            loadStatus += ", " + std::to_string(unsupportedSoundNodeCount) + " unsupported sound XML nodes";
        }
        if (missingSampleCount > 0) {
            loadStatus += ", " + std::to_string(missingSampleCount) + " missing samples";
        }
        return true;
    }

    void SeedDefaults() {
        tree.SetBool("/sim/sound/enabled", true);
        tree.SetBool("/sim/sound/working", true);
        tree.SetDouble("/sim/sound/volume", 1.0);
        tree.SetDouble("/sim/sound/effects/volume", 1.0);
        tree.SetDouble("/sim/sound/Ovolume", 0.45);
        tree.SetDouble("/sim/sound/category/master", 1.0);
        tree.SetDouble("/sim/sound/category/engine", 1.0);
        tree.SetDouble("/sim/sound/category/avionics", 1.0);
        tree.SetDouble("/sim/sound/category/callout", 1.0);
        tree.SetDouble("/sim/sound/category/environment", 1.0);
        tree.SetDouble("/sim/sound/category/announcement", 1.0);
        tree.SetDouble("/sim/sound/category/effects", 1.0);
        tree.SetBool("/sim/current-view/internal", false);
        tree.SetDouble("/sim/current-view/z-offset-m", 12.0);
        tree.SetDouble("/systems/electrical/outputs/efis", 1.0);
        tree.SetDouble("/systems/electrical/outputs/eicas", 1.0);
        tree.SetDouble("/surface-positions/flap-pos-norm", 0.0);
        tree.SetDouble("/gear/gear[0]/position-norm", 0.0);
        tree.SetDouble("/gear/gear[1]/position-norm", 0.0);
        tree.SetDouble("/gear/gear[2]/position-norm", 0.0);
    }

    bool LoadFilters(PropertyListLoader&, const std::filesystem::path& path, std::string& error) {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
            error = "Could not read FlightGear sound filter XML: " + path.string();
            return false;
        }
        const tinyxml2::XMLElement* root = doc.RootElement();
        for (const tinyxml2::XMLElement* filter = root ? root->FirstChildElement("filter") : nullptr; filter; filter = filter->NextSiblingElement("filter")) {
            SystemFilter out{};
            out.gain = ElementDouble(filter, "gain", 1.0);
            const tinyxml2::XMLElement* expression = nullptr;
            if (const tinyxml2::XMLElement* input = filter->FirstChildElement("input")) {
                expression = input->FirstChildElement("expression");
            }
            out.input = expression ? ParseExpression(expression) : nullptr;
            if (const tinyxml2::XMLElement* output = filter->FirstChildElement("output")) {
                out.outputProperty = ElementText(output, "prop");
            }
            if (out.input && !out.outputProperty.empty()) {
                filters.push_back(std::move(out));
            }
        }
        return true;
    }

    bool LoadSounds(PropertyListLoader& loader, const std::filesystem::path& path, std::string& error) {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
            error = "Could not read FlightGear sound XML: " + path.string();
            return false;
        }
        const tinyxml2::XMLElement* fx = doc.RootElement() ? doc.RootElement()->FirstChildElement("fx") : nullptr;
        if (!fx) {
            error = "FlightGear sound XML has no <fx>: " + path.string();
            return false;
        }
        unsupportedSoundNodeCount = 0;
        missingSampleCount = 0;
        const std::set<std::string> supportedChildNames{
            "name",
            "mode",
            "path",
            "condition",
            "property",
            "delay-sec",
            "volume",
            "pitch",
            "position",
            "orientation",
            "reference-dist",
            "max-dist",
            "type",
        };
        for (const tinyxml2::XMLElement* node = fx->FirstChildElement(); node; node = node->NextSiblingElement()) {
            SoundEvent event{};
            event.category = node->Name();
            event.name = ElementText(node, "name", node->Name());
            event.pathText = ElementText(node, "path");
            event.type = ElementText(node, "type", "fx");
            std::set<std::string> unsupportedNames;
            for (const tinyxml2::XMLElement* child = node->FirstChildElement(); child; child = child->NextSiblingElement()) {
                const std::string childName = child->Name();
                if (supportedChildNames.find(childName) == supportedChildNames.end()) {
                    unsupportedNames.insert(childName);
                }
            }
            event.unsupportedNodes.assign(unsupportedNames.begin(), unsupportedNames.end());
            unsupportedSoundNodeCount += static_cast<int>(event.unsupportedNodes.size());
            const std::string mode = ElementText(node, "mode", "once");
            if (mode == "looped") {
                event.mode = SoundEvent::Mode::Looped;
            } else if (mode == "in-transit") {
                event.mode = SoundEvent::Mode::InTransit;
            }
            event.triggerProperty = ElementText(node, "property");
            if (const tinyxml2::XMLElement* condition = node->FirstChildElement("condition")) {
                event.condition = ParseConditionNode(condition);
            }
            event.delay = ElementDouble(node, "delay-sec", 0.0);
            for (const tinyxml2::XMLElement* volume = node->FirstChildElement("volume"); volume; volume = volume->NextSiblingElement("volume")) {
                event.volumes.push_back(ParseSoundParam(volume, false, rng));
            }
            for (const tinyxml2::XMLElement* pitch = node->FirstChildElement("pitch"); pitch; pitch = pitch->NextSiblingElement("pitch")) {
                event.pitches.push_back(ParseSoundParam(pitch, true, rng));
            }
            if (const tinyxml2::XMLElement* pos = node->FirstChildElement("position")) {
                event.position = ParseSoundPosition(pos);
            }
            if (const tinyxml2::XMLElement* orientation = node->FirstChildElement("orientation")) {
                event.orientation = ParseSoundOrientation(orientation);
            }

            const std::filesystem::path soundPath = loader.ResolvePath(event.pathText, path.parent_path());
            std::string loadError;
            const auto buffer = audio.LoadBuffer(soundPath, loadError);
            if (buffer) {
                const auto looped = event.mode != SoundEvent::Mode::Once;
                const float ref = static_cast<float>(ElementDouble(node, "reference-dist", 60.0));
                const float max = static_cast<float>(ElementDouble(node, "max-dist", 6000.0));
                event.source = audio.CreateSource(
                    *buffer,
                    looped,
                    ref,
                    max,
                    static_cast<float>(event.position.x.Eval(tree)),
                    static_cast<float>(event.position.y.Eval(tree)),
                    static_cast<float>(event.position.z.Eval(tree)));
                event.orientation.Apply(event.source, tree);
                event.loaded = true;
                if (!event.triggerProperty.empty()) {
                    event.previousPropertyValue = tree.GetDouble(event.triggerProperty);
                }
            } else {
                event.loaded = false;
                event.unsupportedNodes.push_back("missing sample: " + event.pathText);
                ++missingSampleCount;
            }
            sounds.push_back(std::move(event));
        }
        return true;
    }

    void UpdateBridge(const FlightSim& sim, bool cockpitView, double dt) {
        elapsed += dt;
        const double airspeedKt = sim.SpeedMps() * kMetersPerSecondToKnots;
        const double altitudeFt = sim.AltitudeMeters() * kMetersToFeet;
        const double n1 = std::clamp(18.0 + airspeedKt * 0.16, 0.0, 96.0);
        const bool onGround = sim.AltitudeMeters() < 12.0;
        tree.SetDouble("/sim/time/elapsed-sec", elapsed);
        tree.SetDouble("/velocities/airspeed-kt", airspeedKt);
        tree.SetDouble("/velocities/groundspeed-kt", onGround ? airspeedKt : std::max(0.0, airspeedKt * std::cos(sim.PitchRad())));
        tree.SetDouble("/position/altitude-ft", altitudeFt);
        tree.SetDouble("/position/altitude-agl-ft", altitudeFt);
        tree.SetDouble("/orientation/pitch-deg", RadToDeg(sim.PitchRad()));
        tree.SetDouble("/orientation/roll-deg", RadToDeg(sim.RollRad()));
        tree.SetDouble("/orientation/heading-deg", sim.HeadingDeg());
        tree.SetDouble("/engines/engine[0]/n1", n1);
        tree.SetDouble("/engines/engine[1]/n1", n1);
        tree.SetDouble("/it-autoflight/internal/lookahead-5-sec-airspeed-kt", airspeedKt + 3.0);
        tree.SetBool("/gear/gear[0]/wow", onGround);
        tree.SetBool("/gear/gear[1]/wow", onGround);
        tree.SetBool("/gear/gear[2]/wow", onGround);
        tree.SetBool("/sim/current-view/internal", cockpitView);
        tree.SetDouble("/sim/current-view/z-offset-m", cockpitView ? -3.0 : 12.0);
    }

    void Update(const FlightSim& sim, bool aircraftActive, bool cockpitView, bool enabled, double dt) {
        UpdateBridge(sim, cockpitView, dt);
        for (const SystemFilter& filter : filters) {
            filter.Update(tree);
        }
        const bool soundEnabled = enabled && aircraftActive && tree.GetBool("/sim/sound/enabled", true);
        audio.UpdateListener(static_cast<float>(soundEnabled ? tree.GetDouble("/sim/sound/volume", 1.0) : 0.0));
        if (!soundEnabled) {
            for (SoundEvent& sound : sounds) {
                sound.Stop();
            }
            return;
        }
        for (SoundEvent& sound : sounds) {
            sound.Update(tree, dt);
        }
    }
};

FgRuntime::FgRuntime() = default;
FgRuntime::~FgRuntime() = default;

bool FgRuntime::Initialize(const std::filesystem::path& aircraftRoot, std::string& error) {
    Shutdown();
    m_impl = std::make_unique<Impl>();
    if (!m_impl->Initialize(aircraftRoot, error)) {
        m_status = "FlightGear audio failed: " + error;
        m_impl.reset();
        return false;
    }
    m_status = m_impl->loadStatus;
    return true;
}

void FgRuntime::Shutdown() {
    m_impl.reset();
    m_status = "FlightGear audio: not initialized";
}

void FgRuntime::Reset() {
    if (m_impl) {
        for (SoundEvent& sound : m_impl->sounds) {
            sound.Stop();
        }
    }
}

bool FgRuntime::IsReady() const {
    return m_impl != nullptr;
}

void FgRuntime::SetEnabled(bool enabled) {
    m_enabled = enabled;
    if (!enabled) {
        Reset();
    }
}

void FgRuntime::UpdateFromSim(const FlightSim& sim, bool aircraftActive, bool cockpitView, double dtSeconds) {
    if (m_impl) {
        m_impl->Update(sim, aircraftActive, cockpitView, m_enabled, dtSeconds);
    }
}

std::vector<FgSoundDebugInfo> FgRuntime::SoundDebugSnapshot() const {
    std::vector<FgSoundDebugInfo> out;
    if (!m_impl) {
        return out;
    }
    out.reserve(m_impl->sounds.size());
    for (const SoundEvent& sound : m_impl->sounds) {
        out.push_back({
            sound.name,
            sound.pathText,
            sound.category,
            sound.type,
            JoinStrings(sound.unsupportedNodes, ", "),
            sound.loaded,
            sound.IsPlaying(),
            sound.lastCondition,
            sound.lastVolume,
            sound.lastPitch,
        });
    }
    return out;
}

std::vector<FgPropertyDebugInfo> FgRuntime::FindProperties(const std::string& query, size_t limit) const {
    return m_impl ? m_impl->tree.Find(query, limit) : std::vector<FgPropertyDebugInfo>{};
}

bool FgRuntime::SetPropertyFromUi(const std::string& path, const std::string& valueText) {
    if (!m_impl) {
        return false;
    }
    m_impl->tree.SetFromText(path, valueText, "");
    return true;
}

bool FgRuntime::SetPropertyDouble(const std::string& path, double value) {
    if (!m_impl) {
        return false;
    }
    m_impl->tree.SetDouble(path, value);
    return true;
}

} // namespace flight
