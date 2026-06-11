/**
* @brief     Generic string manipulation utilities.
*/

#ifndef AVT_341_STRING_UTILS_H
#define AVT_341_STRING_UTILS_H

#include <algorithm>
#include <cctype>
#include <string>

namespace avt_341::core
{

/** @brief Return a lowercase copy of the given string. */
inline std::string ToLowerCase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    return value;
}

/** @brief Sanitize a string into a lowercase identifier: the input is
 *         lowercased and every character outside [a-z0-9_] is replaced
 *         with @p replacement. Suitable for ROS topic/namespace tokens,
 *         logger names and TF frame segments. */
inline std::string SanitizeIdentifier(const std::string& value,
                                      const char replacement = '_') {
    std::string sanitized = ToLowerCase(value);
    for (auto& character : sanitized) {
        const bool valid = (character >= 'a' && character <= 'z') ||
                           (character >= '0' && character <= '9') ||
                           character == '_';
        if (!valid) {
            character = replacement;
        }
    }
    return sanitized;
}

}

#endif  // AVT_341_STRING_UTILS_H
