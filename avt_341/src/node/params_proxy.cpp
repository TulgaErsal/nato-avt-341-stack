#include <avt_341/node/params_proxy.h>

namespace avt_341::node {

// ====================================================================================================================
// SHARED - PARAMETER EVENT
// ====================================================================================================================

bool RosParameterEvent::contains_parameter(const std::string &parameter_name) const {
    return parameter_map_.find(parameter_name) != parameter_map_.end();
}

RosParameter* RosParameterEvent::get_parameter(const std::string &parameter_name) const {
    return contains_parameter(parameter_name) ? parameter_map_.at(parameter_name) : nullptr;
}

RosParameterEvent::RosParameterEvent(const std::vector<RosParameter> &changed_params, const std::string &node_name)
    : node(node_name), changed_parameters(changed_params) {

    cache_parameter_map();
}

void RosParameterEvent::cache_parameter_map() {
    parameter_map_.clear();
    for (auto & p : changed_parameters) {
        parameter_map_[p.get_name()] = &p;
    }
}

#ifdef ROS1

// ====================================================================================================================
// ROS1 - BASE PARAMETER
// ====================================================================================================================

const std::string & RosParameter::get_name() const {
    static const std::string empty_name;
    return empty_name;
}

std::string RosParameter::get_type_name() const {
    return "";
}

bool RosParameter::as_bool() const {
    return false;
}

int64_t RosParameter::as_int() const {
    return 0;
}

double RosParameter::as_double() const {
    return 0.0;
}

const std::string & RosParameter::as_string() const {
    static const std::string empty_string;
    return empty_string;
}

const std::vector<std::string> & RosParameter::as_string_array() const {
    static const std::vector<std::string> empty_array;
    return empty_array;
}

// ====================================================================================================================
// ROS1 - PARAMETER PROXY
// ====================================================================================================================

void ParamsProxy::add_parameter_callback(
    const std::string &parameter_name,
    const std::function<void(const RosParameter &)>& callback,
    const std::string &node_name) {

    // No ROS1 Implementation
}

void ParamsProxy::add_parameter_callback(
    const std::vector<std::string> &parameter_names,
    const std::function<void(const RosParameterEvent &)> &callback,
    const std::string &node_name) {

    // No ROS1 Implementation
}

void ParamsProxy::add_parameter_event_callback(
    const std::function<void(const RosParameterEvent &)>& callback) {

    // No ROS1 Implementation
}

#else

// ====================================================================================================================
// ROS2 - BASE PARAMETER
// ====================================================================================================================

RosParameter::RosParameter(const rclcpp::Parameter & param) : param_(param){ }

const std::string & RosParameter::get_name() const { return param_.get_name(); }
std::string RosParameter::get_type_name() const { return param_.get_type_name();}

bool RosParameter::as_bool() const { return param_.as_bool(); }
int64_t RosParameter::as_int() const { return param_.as_int(); }
double RosParameter::as_double() const { return param_.as_double(); }
const std::string & RosParameter::as_string() const { return param_.as_string(); }
const std::vector<std::string> & RosParameter::as_string_array() const { return param_.as_string_array(); }

// ====================================================================================================================
// ROS2 - PARAMETER EVENT
// ====================================================================================================================

RosParameterEvent::RosParameterEvent(const rcl_interfaces::msg::ParameterEvent &param_event)
    : node(param_event.node) {

    changed_parameters.reserve(param_event.changed_parameters.size());

    for (const auto & p : param_event.changed_parameters) {
        changed_parameters.emplace_back(rclcpp::Parameter::from_parameter_msg(p));
    }

    cache_parameter_map();
}

// ====================================================================================================================
// ROS2 - PARAMETER PROXY
// ====================================================================================================================

ParamsProxy::ParamsProxy(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);

    own_node_name_ = std::string(node_->get_namespace()) + "/" + std::string(node_->get_name());
    if (own_node_name_[0] != '/') {
        own_node_name_ = "/" + own_node_name_;
    }
}

void ParamsProxy::add_parameter_callback(
    const std::string &parameter_name,
    const std::function<void(const RosParameter &)>& callback,
    const std::string &node_name) {

    const auto handler = param_subscriber_->add_parameter_callback(
        parameter_name,
        [callback](const rclcpp::Parameter & p){ callback(RosParameter(p)); },
        node_name
        );

    client_param_handlers_.push_back(handler);
}

void ParamsProxy::add_parameter_event_callback(
    const std::function<void(const RosParameterEvent &)>& callback) {

    const auto handler = param_subscriber_->add_parameter_event_callback(
        [callback](const rcl_interfaces::msg::ParameterEvent & pe){
            callback(RosParameterEvent(pe));
        }
    );

    client_event_handlers_.push_back(handler);
}

void ParamsProxy::on_multi_param_preprocess(const rcl_interfaces::msg::ParameterEvent & pe) {

    RosParameterEvent param_event(pe);

    for (const auto & context : multi_param_callback_contexts_) {

        const bool select_own_node = context.node_name.empty();
        if ((select_own_node && param_event.node != own_node_name_)
            || (!select_own_node && context.node_name != param_event.node)) {
            continue;
        }

        std::vector<RosParameter> found_params;
        for (const auto & param_name : context.parameter_names) {
            if (param_event.contains_parameter(param_name)) {
                found_params.push_back(*param_event.get_parameter(param_name));
            }
        }

        context.callback(RosParameterEvent(found_params, param_event.node));
    }
}


void ParamsProxy::add_parameter_callback(
    const std::vector<std::string> &parameter_names,
    const std::function<void(const RosParameterEvent &)> &callback,
    const std::string &node_name) {

    if (parameter_names.empty()) {
        return;
    }

    if (event_cb_handle_ == nullptr) {
        event_cb_handle_ = param_subscriber_->add_parameter_event_callback(
            std::bind(&ParamsProxy::on_multi_param_preprocess, this, std::placeholders::_1)
            );
    }

    std::string target_node = node_name;
    if (!node_name.empty() && node_name[0] != '/') {
        target_node = "/" + target_node;
    }
    multi_param_callback_contexts_.emplace_back(parameter_names, callback, target_node);
}


#endif

}
