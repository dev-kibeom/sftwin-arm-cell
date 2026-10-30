#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>

namespace arm_cell_motion
{

// Runs one candidate at a time. Child 0 selects a candidate, child 1 attempts
// it, and child 2 authorizes another attempt after a failed attempt. Exhaustion
// and a denied transition remain FAILURE; neither is hidden from the parent.
class SearchCandidates final : public BT::ControlNode
{
public:
  SearchCandidates(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    transition_running_ = false;
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 3) {
      throw BT::BehaviorTreeException(
              "SearchCandidates requires selector, attempt, and transition children");
    }

    for (;; ) {
      if (!transition_running_) {
        const auto selected = children_nodes_[0]->executeTick();
        if (selected == BT::NodeStatus::RUNNING) {return selected;}
        if (selected == BT::NodeStatus::FAILURE) {
          haltChildren();
          return BT::NodeStatus::FAILURE;
        }

        const auto attempted = children_nodes_[1]->executeTick();
        if (attempted == BT::NodeStatus::RUNNING) {return attempted;}
        if (attempted == BT::NodeStatus::SUCCESS) {
          haltChildren();
          return BT::NodeStatus::SUCCESS;
        }
        transition_running_ = true;
      }

      const auto may_continue = children_nodes_[2]->executeTick();
      if (may_continue == BT::NodeStatus::RUNNING) {return may_continue;}
      transition_running_ = false;
      if (may_continue == BT::NodeStatus::FAILURE) {
        haltChildren();
        return BT::NodeStatus::FAILURE;
      }

      resetChildren();
    }
  }

  bool transition_running_{false};
};

// Repeats an attempt only when its explicit transition child authorizes it.
// A denied transition terminates as FAILURE; the failed attempt is never
// translated into SUCCESS to satisfy a Repeat decorator.
class RetryWhileAllowed final : public BT::ControlNode
{
public:
  RetryWhileAllowed(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    transition_running_ = false;
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::BehaviorTreeException(
              "RetryWhileAllowed requires attempt and transition children");
    }

    for (;; ) {
      if (!transition_running_) {
        const auto attempted = children_nodes_[0]->executeTick();
        if (attempted == BT::NodeStatus::RUNNING) {return attempted;}
        if (attempted == BT::NodeStatus::SUCCESS) {
          haltChildren();
          return BT::NodeStatus::SUCCESS;
        }
        transition_running_ = true;
      }
      const auto may_retry = children_nodes_[1]->executeTick();
      if (may_retry == BT::NodeStatus::RUNNING) {return may_retry;}
      transition_running_ = false;
      if (may_retry == BT::NodeStatus::FAILURE) {
        haltChildren();
        return BT::NodeStatus::FAILURE;
      }
      resetChildren();
    }
  }

  bool transition_running_{false};
};

// Executes a recovery child after a failed primary child while preserving the
// primary FAILURE as the control result. A successful recovery describes safe
// cleanup; it does not turn the original operation into a domain success.
class RecoverOnFailure final : public BT::ControlNode
{
public:
  RecoverOnFailure(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    recovery_started_ = false;
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::BehaviorTreeException(
              "RecoverOnFailure requires primary and recovery children");
    }
    if (!recovery_started_) {
      const auto primary = children_nodes_[0]->executeTick();
      if (primary == BT::NodeStatus::RUNNING) {return primary;}
      if (primary == BT::NodeStatus::SUCCESS) {
        haltChildren();
        return primary;
      }
      recovery_started_ = true;
    }
    const auto recovered = children_nodes_[1]->executeTick();
    if (recovered == BT::NodeStatus::RUNNING) {return recovered;}
    recovery_started_ = false;
    haltChildren();
    return BT::NodeStatus::FAILURE;
  }

  bool recovery_started_{false};
};

// Runs its finalizer after either terminal primary result and returns the
// primary status unchanged. It is used for per-attempt diagnostics and state
// recording, whose completion must not rewrite task success or failure.
class RunAndFinalize final : public BT::ControlNode
{
public:
  RunAndFinalize(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    primary_finished_ = false;
    primary_status_ = BT::NodeStatus::IDLE;
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::BehaviorTreeException(
              "RunAndFinalize requires primary and finalizer children");
    }
    if (!primary_finished_) {
      primary_status_ = children_nodes_[0]->executeTick();
      if (primary_status_ == BT::NodeStatus::RUNNING) {return primary_status_;}
      primary_finished_ = true;
    }
    const auto finalized = children_nodes_[1]->executeTick();
    if (finalized == BT::NodeStatus::RUNNING) {return finalized;}
    const auto result = primary_status_;
    primary_finished_ = false;
    primary_status_ = BT::NodeStatus::IDLE;
    haltChildren();
    return result;
  }

  bool primary_finished_{false};
  BT::NodeStatus primary_status_{BT::NodeStatus::IDLE};
};

inline void register_candidate_search_control(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<SearchCandidates>("SearchCandidates");
  factory.registerNodeType<RetryWhileAllowed>("RetryWhileAllowed");
  factory.registerNodeType<RecoverOnFailure>("RecoverOnFailure");
  factory.registerNodeType<RunAndFinalize>("RunAndFinalize");
}

}  // namespace arm_cell_motion
