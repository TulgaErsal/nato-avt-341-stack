#ifndef AVT_341_PARAMS_PROXY_H
#define AVT_341_PARAMS_PROXY_H

#include <unordered_map>

#ifdef ROS_1
#include <ros/ros.h>
#include <boost/optional/optional.hpp>
#else
#include <rclcpp/rclcpp.hpp>
#endif

namespace avt_341 {

/**
 * Alias for an optional value type used for ROS 1 and ROS 2 builds respectively.
 * std::optional not supported in C++14 traditionally targeted by ROS1.
 */
#ifdef ROS_1
template<typename T>
using optional = boost::optional<T>;
#else
template<typename T>
using optional = std::optional<T>;
#endif

}

namespace avt_341::node {

/**
* @brief Base ROS parameter type.
 * Wraps concrete ROS1 or ROS2 parameter so that version is transparent to user.
 */
class RosParameter {

public:

#ifndef ROS_1
    /**
     * @brief Construct a RosParameter from an underlying rclcpp::Parameter.
     *
     * @param param The underlying ROS parameter to wrap.
     */
    explicit RosParameter(const rclcpp::Parameter & param);
#endif

    /**
     * @brief Get the fully-qualified name of the parameter.
     *
     * @return Constant reference to the parameter name.
     */
    const std::string & get_name() const;

    /**
     * @brief Get the type of the parameter as a human-readable string.
     *
     * The returned string reflects the underlying ROS parameter type, such as
     * "bool", "integer", "double", or "string".
     *
     * @return Parameter type name.
     */
    std::string get_type_name() const;

    /**
     * @brief Get the parameter value as the requested type.
     *
     * @tparam T Desired C++ type of the returned value.
     * @return The parameter value converted to type @p T.
     */
    template<typename T>
    decltype(auto) get_value() const {
        #ifdef ROS_1
        return T();
        #else
        return param_.get_value<T>();
        #endif
    }

    /**
     * @brief Get the parameter value as a bool.
     *
     * @return The parameter interpreted as a boolean value.
     */
    bool as_bool() const;

    /**
     * @brief Get the parameter value as a 64-bit integer.
     *
     * @return The parameter interpreted as a signed 64-bit integer.
     */
    int64_t as_int() const;

    /**
     * @brief Get the parameter value as a double.
     *
     * @return The parameter interpreted as a double-precision floating-point value.
     */
    double as_double() const;

    /**
     * @brief Get the parameter value as a string.
     *
     * @return Constant reference to the parameter interpreted as a string.
     */
    const std::string & as_string() const;

    /**
     * @brief Get the parameter value as an array of strings.
     *
     * @return Constant reference to the parameter interpreted as a vector of strings.
     */
    const std::vector<std::string> & as_string_array() const;

private:

#ifndef ROS_1
    /// Underlying ROS parameter instance.
    const rclcpp::Parameter param_;
#endif

};

/**
 * @brief Aggregates a set of parameter changes for a single node. Supports atomic setting.
 * Wraps concrete ROS1 or ROS2 parameter event so that version is transparent to user.
 *
 */
class RosParameterEvent {

public:

#ifndef ROS_1
    /**
     * @brief Construct a RosParameterEvent from an underlying ROS parameter event message.
     *
     * @param param_event The underlying rcl_interfaces::msg::ParameterEvent
     *                    message describing the change.
     */
    explicit RosParameterEvent(const rcl_interfaces::msg::ParameterEvent & param_event);
#endif

    /**
     * @brief Construct a RosParameterEvent from a list of changed parameters.
     *
     * @param changed_params List of parameters that have changed.
     * @param node_name Name of the node that owns the parameters.
     */
    explicit RosParameterEvent(const std::vector<RosParameter> & changed_params, const std::string &node_name);

    /// Fully-qualified name of the node whose parameters changed in the parameter event.
    std::string node;

    /// Parameters that were added, modified, or deleted in this event.
    std::vector<RosParameter> changed_parameters;

    /**
     * @brief Check whether the event contains a parameter with the given name.
     *
     * @param parameter_name Name of the parameter to search for.
     * @return true if a parameter with the specified name exists in the event, false otherwise.
     */
    bool contains_parameter(const std::string & parameter_name) const;

    /**
     * @brief Get a pointer to the parameter with the given name.
     *
     * @param parameter_name Name of the parameter to retrieve.
     * @return Pointer to the corresponding RosParameter if found, otherwise nullptr.
     */
    RosParameter* get_parameter(const std::string & parameter_name) const;

    /**
     * @brief Get the value of a parameter in this event as the requested type.
     *
     * If the parameter is present, this returns an engaged optional containing
     * the value converted to type @p T. If the parameter is not present, an
     * empty optional is returned.
     *
     * @tparam T Desired C++ type of the returned value.
     * @param parameter_name Name of the parameter whose value should be read.
     * @return Optional containing the parameter value if it exists, or an empty
     *         optional if the parameter is not part of this event.
     */
    template<typename T>
    optional<T> get_value(const std::string &parameter_name) const {
        return contains_parameter(parameter_name) ? parameter_map_.at(parameter_name)->get_value<T>() : optional<T>();
    }

private:
    /// Populate the internal name-to-parameter lookup map from changed_parameters.
    void cache_parameter_map();

    /// Map from parameter name to the corresponding RosParameter pointer.
    std::unordered_map<std::string, RosParameter*> parameter_map_;
};

/**
 * @brief Management class for handling ROS parameters.
 * Abstract or wraps ROS1 and ROS2 implementation details so that transparent to client.
 */
class ParamsProxy {

public:

    /**
     * @brief Context information for callbacks that depend on multiple parameters.
     *
     * Stores the set of parameter names a callback is interested in, the
     * callback itself, and the associated node name.
     */
    struct ParameterCallbackContext {

        /**
         * @brief Construct a context object for a multi-parameter callback.
         *
         * @param parameter_names List of parameter names this callback depends on.
         * @param callback Callback to invoke when one or more of the parameters change.
         * @param node_name Name of the node that owns the parameters.
         */
        ParameterCallbackContext(const std::vector<std::string> &parameter_names,
            const std::function<void(const RosParameterEvent &)> &callback, const std::string &node_name)
            : parameter_names(parameter_names),
              callback(callback),
              node_name(node_name) {
        }

        /// Names of parameters the callback depends on.
        std::vector<std::string> parameter_names;

        /// Callback to invoke when the corresponding parameters change.
        std::function<void (const RosParameterEvent &)> callback;

        /// Name of the node associated with this callback context.
        std::string node_name;
    };

#ifndef ROS_1
    /**
     * @brief Construct a ParamsProxy bound to an underlying ROS node.
     *
     * @param node Shared pointer to the rclcpp::Node whose parameters are to be observed.
     */
    explicit ParamsProxy(const std::shared_ptr<rclcpp::Node>& node);
#endif

    /**
     * @brief Register a callback for a single parameter.
     *
     * The callback is invoked whenever the specified parameter changes.
     *
     * @param parameter_name Name of the parameter to monitor.
     * @param callback Function to call when the parameter is updated.
     * @param node_name Optional name of the node that owns the parameter. If empty,
     *                  the proxy's own node name is used.
     */
    void add_parameter_callback(
        const std::string & parameter_name,
        const std::function<void (const RosParameter &)>& callback,
        const std::string & node_name = ""
    );

    /**
     * @brief Register a callback that depends on multiple parameters.
     *
     * The callback is invoked with a RosParameterEvent whenever any of the
     * specified parameters are changed.
     *
     * @param parameter_names List of parameter names that should trigger the callback.
     * @param callback Function to call when one or more of the parameters change.
     * @param node_name Optional name of the node that owns the parameters. If empty,
     *                  the proxy's own node name is used.
     */
    void add_parameter_callback(
        const std::vector<std::string> & parameter_names,
        const std::function<void (const RosParameterEvent &)>& callback,
        const std::string & node_name = ""
    );

    /**
     * @brief Register a callback for raw parameter events.
     *
     * The callback is invoked for every parameter event received by the node,
     * regardless of which parameters were affected.
     *
     * @param callback Function to call when any parameter event is received.
     */
    void add_parameter_event_callback(
        const std::function<void (const RosParameterEvent &)> &callback
    );

private:

    /// Contexts for callbacks that depend on multiple parameters.
    std::vector<ParameterCallbackContext> multi_param_callback_contexts_;

    /// Name of the node that owns this ParamsProxy instance.
    std::string own_node_name_;

#ifndef ROS_1
    /**
     * @brief Pre-process a raw ROS ParameterEvent before dispatching callbacks.
     *
     * This builds a RosParameterEvent and triggers any registered multi-parameter
     * callbacks that are affected by the event.
     *
     * @param pe Incoming ROS parameter event message.
     */
    void on_multi_param_preprocess(const rcl_interfaces::msg::ParameterEvent & pe);

    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

    /// Handles for client parameter registered callbacks.
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> client_param_handlers_;

    /// Handles for client parameter event registered callbacks.
    std::vector<std::shared_ptr<rclcpp::ParameterEventCallbackHandle>> client_event_handlers_;

    /// Handle for the internal parameter event callback used for preprocessing when multi-parameter changes subscribed to.
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> event_cb_handle_;
#endif

};


}

#endif //AVT_341_PARAMS_PROXY_H