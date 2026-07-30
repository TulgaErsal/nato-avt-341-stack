/**
* @brief     Generic string manipulation utilities.
*/

#ifndef AVT_341_STRING_UTILS_H
#define AVT_341_STRING_UTILS_H

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace avt_341_nav::core
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

/** @brief GIS crs identifier to tf frame id, e.g. "EPSG:6495" -> "epsg_6495". */
inline std::string CrsToFrameId(const std::string& crs) {
    return SanitizeIdentifier(crs);
}

/// Convert an integer to a string with zero padding
inline std::string IntToString(int x, int zero_padding){
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(zero_padding) << x;
  std::string str = ss.str();
  return str;
};

/**
 * @brief Trims the input string, removing all leading and trailing characters that match
 * the specified character (default is space).
 *
 * @param str String to be trimmed.
 * @param char_to_remove Character to remove.
 * @return Trimmed string.
 */
inline std::string Trim(const std::string& str, const char char_to_remove = ' ')
{
    const auto start = str.find_first_not_of(char_to_remove);
    if (start == std::string::npos) return "";
    const auto end = str.find_last_not_of(char_to_remove);
    return str.substr(start, end - start + 1);
}

/**
 * @brief Split a string with a specified delimiter.
 *
 * @param str String to be split.
 * @param delimiter Delimiter character used to split the string.
 * @param trim_whitespace If set, trims whitespaces from split substrings.
 * @return std::vector<std::string> Vector of split substrings (excluding the delimiter).
 */
inline std::vector<std::string> SplitByDelimiter(
    const std::string& str,
    const char delimiter = '-',
    const bool trim_whitespace = true
    ){
    std::stringstream stream(str);
    std::vector<std::string> tokens;
    std::string token;
    while(std::getline(stream, token, delimiter)) { tokens.push_back(trim_whitespace ? Trim(token) : token); }
    return tokens;
}

}

#endif  // AVT_341_STRING_UTILS_H
