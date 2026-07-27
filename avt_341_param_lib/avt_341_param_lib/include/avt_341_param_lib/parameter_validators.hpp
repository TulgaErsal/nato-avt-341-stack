#ifndef AVT_341_PARAM_LIB_PARAMETER_VALIDATORS_H
#define AVT_341_PARAM_LIB_PARAMETER_VALIDATORS_H

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
// Only rclcpp::Parameter and rclcpp::ParameterType are used here; pulling in
// rclcpp/node.hpp (or rclcpp_lifecycle) would drag the whole pub/sub/service
// template machinery into every translation unit that validates a parameter.
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <optional>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

namespace avt_341_param_lib {

// Validation result type: nullopt = success, string value = error message
using ValidationResult = std::optional<std::string>;

// Algorithm helpers
template <typename Collection>
[[nodiscard]] auto contains(Collection const& collection,
                            typename Collection::const_reference value) {
    return std::find(collection.cbegin(), collection.cend(), value) != collection.cend();
}

template <typename Collection>
[[nodiscard]] auto is_unique(Collection collection) {
    std::sort(collection.begin(), collection.end());
    return std::adjacent_find(collection.cbegin(), collection.cend()) == collection.cend();
}

namespace detail {
template <typename T>
[[nodiscard]] auto stringify(T const& value) -> std::string {
    if constexpr (std::is_floating_point_v<T>) return fmt::format("{:g}", value);
    return fmt::format("{}", value);
}

template <typename T>
[[nodiscard]] auto join_stringified(std::vector<T> const& values) -> std::string {
    std::vector<std::string> tokens;
    tokens.reserve(values.size());
    for (auto const& value : values) tokens.push_back(stringify(value));
    return fmt::format("{}", fmt::join(tokens, ", "));
}

template <typename T, typename Fn>
[[nodiscard]] auto size_compare(rclcpp::Parameter const& parameter, size_t const size,
                                std::string const& predicate_description,
                                Fn const& predicate) -> ValidationResult {
    static constexpr auto format_string = "Length of parameter '{}' is '{}' but must be {} '{}'";
    switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_STRING:
            if (auto value = parameter.get_value<std::string>(); !predicate(value.size(), size))
                return fmt::format(format_string, parameter.get_name(), value.size(),
                                   predicate_description, size);
            break;
        default:
            if (auto value = parameter.get_value<std::vector<T>>(); !predicate(value.size(), size))
                return fmt::format(format_string, parameter.get_name(), value.size(),
                                   predicate_description, size);
    }
    return std::nullopt;
}

template <typename T, typename Fn>
[[nodiscard]] auto compare(rclcpp::Parameter const& parameter, T const& value,
                           std::string const& predicate_description,
                           Fn const& predicate) -> ValidationResult {
    if (auto const param_value = parameter.get_value<T>(); !predicate(param_value, value))
        return fmt::format("Parameter '{}' with the value '{}' must be {} '{}'",
                           parameter.get_name(), stringify(param_value),
                           predicate_description, stringify(value));
    return std::nullopt;
}
}  // namespace detail

// Parameter validators
template <typename T>
[[nodiscard]] auto unique(rclcpp::Parameter const& parameter) -> ValidationResult {
    if (is_unique(parameter.get_value<std::vector<T>>())) return std::nullopt;
    return fmt::format("Parameter '{}' must only contain unique values", parameter.get_name());
}

template <typename T>
[[nodiscard]] auto subset_of(rclcpp::Parameter const& parameter,
                             std::vector<T> const& valid_values) -> ValidationResult {
    auto const& values = parameter.get_value<std::vector<T>>();
    for (auto const& value : values)
        if (!contains(valid_values, value))
            return fmt::format("Entry '{}' in parameter '{}' is not in the set '{{{}}}'", value,
                               parameter.get_name(), fmt::join(valid_values, ", "));
    return std::nullopt;
}

template <typename T>
[[nodiscard]] auto fixed_size(rclcpp::Parameter const& parameter, size_t const size) {
    return detail::size_compare<T>(parameter, size, "equal to", std::equal_to<>());
}

template <typename T>
[[nodiscard]] auto size_gt(rclcpp::Parameter const& parameter, size_t const size) {
    return detail::size_compare<T>(parameter, size, "greater than", std::greater<>());
}

template <typename T>
[[nodiscard]] auto size_lt(rclcpp::Parameter const& parameter, size_t const size) {
    return detail::size_compare<T>(parameter, size, "less than", std::less<>());
}

template <typename T>
[[nodiscard]] auto not_empty(rclcpp::Parameter const& parameter) -> ValidationResult {
    switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_STRING:
            if (auto param_value = parameter.get_value<std::string>(); param_value.empty())
                return fmt::format("Parameter '{}' cannot be empty", parameter.get_name());
            break;
        default:
            if (auto param_value = parameter.get_value<std::vector<T>>(); param_value.empty())
                return fmt::format("Parameter '{}' cannot be empty", parameter.get_name());
    }
    return std::nullopt;
}

template <typename T>
[[nodiscard]] auto element_bounds(rclcpp::Parameter const& parameter, T const& lower,
                                  T const& upper) -> ValidationResult {
    auto const& param_value = parameter.get_value<std::vector<T>>();
    for (auto val : param_value)
        if (val < lower || val > upper)
            return fmt::format("Value '{}' in parameter '{}' must be within bounds '[{}, {}]'",
                               detail::stringify(val), parameter.get_name(),
                               detail::stringify(lower), detail::stringify(upper));
    return std::nullopt;
}

template <typename T>
[[nodiscard]] auto lower_element_bounds(rclcpp::Parameter const& parameter,
                                        T const& lower) -> ValidationResult {
    auto const& param_value = parameter.get_value<std::vector<T>>();
    for (auto val : param_value)
        if (val < lower)
            return fmt::format("Value '{}' in parameter '{}' must be above lower bound of '{}'",
                               detail::stringify(val), parameter.get_name(),
                               detail::stringify(lower));
    return std::nullopt;
}

template <typename T>
[[nodiscard]] auto upper_element_bounds(rclcpp::Parameter const& parameter,
                                        T const& upper) -> ValidationResult {
    auto const& param_value = parameter.get_value<std::vector<T>>();
    for (auto val : param_value)
        if (val > upper)
            return fmt::format("Value '{}' in parameter '{}' must be below upper bound of '{}'",
                               detail::stringify(val), parameter.get_name(),
                               detail::stringify(upper));
    return std::nullopt;
}

template <typename T>
[[nodiscard]] auto bounds(rclcpp::Parameter const& parameter, T const& lower,
                          T const& upper) -> ValidationResult {
    auto const& param_value = parameter.get_value<T>();
    if (param_value < lower || param_value > upper)
        return fmt::format("Parameter '{}' with the value '{}' must be within bounds '[{}, {}]'",
                           parameter.get_name(), detail::stringify(param_value),
                           detail::stringify(lower), detail::stringify(upper));
    return std::nullopt;
}

template <typename T>
[[nodiscard]] auto lt(rclcpp::Parameter const& parameter, T const& value) {
    return detail::compare(parameter, value, "less than", std::less<T>());
}

template <typename T>
[[nodiscard]] auto gt(rclcpp::Parameter const& parameter, T const& value) {
    return detail::compare(parameter, value, "greater than", std::greater<T>());
}

template <typename T>
[[nodiscard]] auto lt_eq(rclcpp::Parameter const& parameter, T const& value) {
    return detail::compare(parameter, value, "less than or equal to", std::less_equal<T>());
}

template <typename T>
[[nodiscard]] auto gt_eq(rclcpp::Parameter const& parameter, T const& value) {
    return detail::compare(parameter, value, "greater than or equal to", std::greater_equal<T>());
}

template <typename T>
[[nodiscard]] auto one_of(rclcpp::Parameter const& parameter,
                          std::vector<T> const& collection) -> ValidationResult {
    auto const& param_value = parameter.get_value<T>();
    if (contains(collection, param_value)) return std::nullopt;
    return fmt::format("Parameter '{}' with the value '{}' is not in the set '{{{}}}'",
                       parameter.get_name(), detail::stringify(param_value),
                       detail::join_stringified(collection));
}

[[nodiscard]] auto to_parameter_result_msg(ValidationResult const& result)
    -> rcl_interfaces::msg::SetParametersResult;

}

#endif //AVT_341_PARAM_LIB_PARAMETER_VALIDATORS_H
