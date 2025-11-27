#ifndef AVT_341_PARAMS_PROXY_H
#define AVT_341_PARAMS_PROXY_H

#ifdef ROS1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

namespace avt_341 {

#ifdef ROS1
template<typename T>
using optional = boost::optional<T>;
#else
template<typename T>
using optional = std::optional<T>;
#endif

}

namespace avt_341::node {

class RosParameter {

public:

#ifndef ROS1
    explicit RosParameter(const rclcpp::Parameter & param);
#endif

    const std::string & get_name() const;

    std::string get_type_name() const;

    template<typename T>
    decltype(auto) get_value() const {
        #ifdef ROS1
        return T();
        #else
        return param_.get_value<T>();
        #endif
    }

    bool as_bool() const;
    int64_t as_int() const;
    double as_double() const;
    const std::string & as_string() const;
    const std::vector<std::string> & as_string_array() const;

private:

#ifndef ROS1
    const rclcpp::Parameter param_;
#endif

};

class RosParameterEvent {

public:

#ifndef ROS1
    explicit RosParameterEvent(const rcl_interfaces::msg::ParameterEvent & param_event);
#endif

    explicit RosParameterEvent(const std::vector<RosParameter> & changed_params, const std::string &node_name);

    std::string node;
    std::vector<RosParameter> changed_parameters;
    bool contains_parameter(const std::string & parameter_name) const;
    RosParameter* get_parameter(const std::string & parameter_name) const;

    template<typename T>
    optional<T> get_value(const std::string &parameter_name) const {
        return contains_parameter(parameter_name) ? parameter_map_.at(parameter_name)->get_value<T>() : optional<T>();
    }

private:
    void cache_parameter_map();

    std::unordered_map<std::string, RosParameter*> parameter_map_;
};

class ParamsProxy {

public:

    struct ParameterCallbackContext {

        ParameterCallbackContext(const std::vector<std::string> &parameter_names,
            const std::function<void(const RosParameterEvent &)> &callback, const std::string &node_name)
            : parameter_names(parameter_names),
              callback(callback),
              node_name(node_name) {
        }

        std::vector<std::string> parameter_names;
        std::function<void (const RosParameterEvent &)> callback;
        std::string node_name;
    };

#ifndef ROS1
    explicit ParamsProxy(const std::shared_ptr<rclcpp::Node>& node);
#endif

    void add_parameter_callback(
        const std::string & parameter_name,
        const std::function<void (const RosParameter &)>& callback,
        const std::string & node_name = ""
    );

    void add_parameter_callback(
        const std::vector<std::string> & parameter_names,
        const std::function<void (const RosParameterEvent &)>& callback,
        const std::string & node_name = ""
    );

    void add_parameter_event_callback(
        const std::function<void (const RosParameterEvent &)> &callback
    );

private:

    std::vector<ParameterCallbackContext> multi_param_callback_contexts_;
    std::string own_node_name_;

#ifndef ROS1
    void on_multi_param_preprocess(const rcl_interfaces::msg::ParameterEvent & pe);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> client_param_handlers_;
    std::vector<std::shared_ptr<rclcpp::ParameterEventCallbackHandle>> client_event_handlers_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> event_cb_handle_;
#endif

};


}

#endif //AVT_341_PARAMS_PROXY_H