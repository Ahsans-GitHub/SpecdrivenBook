---
id: lesson3
title: Cognitive Planning Using LLMs to translate natural language Clean the room into a sequence of ROS 2 actions
sidebar_label: Manipulation and grasping with humanoid hands
---

# Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions.

## Heading Breakdown
**Cognitive Planning** is the ability to think ahead. It is the bridge between the abstract desire ("Clean the room") and the concrete steps needed to achieve it. **Using LLMs** leverages the common sense reasoning of models trained on the internet. **To translate natural language** means converting fuzzy human speech into rigid machine code. **Sequence of ROS 2 actions** is the output: a list of calls to Action Servers (e.g., `navigate_to`, `pick_up`, `drop_off`). The importance is **task composition**; we don't have to program "Clean Room"; we program the atomic skills, and the LLM composes them. Real usage involves a **Unitree G1** receiving the command "Clean the room," noticing a sock on the floor, and generating the plan: `[Goto(Sock), Grasp(Sock), Goto(Hamper), Drop(Sock)]`. This is key for **upgradable systems** because adding a new skill (e.g., `IronClothes`) immediately makes it available to the planner.

*(Note: Sidebar refers to Manipulation, but per mapping, we cover Cognitive Planning here).*

## Training Focus: Reasoning
We focus on **logic**.
*   **Decomposition**: Breaking big problems into small ones.
*   **Constraints**: "Don't hold the cup while opening the door."

## Detailed Content
### LangChain for Robots
Using libraries to chain thoughts.
*   **Tools**: Giving the LLM a list of functions it can call (`move_robot(x,y)`).
*   **Agents**: The loop of "Think, Act, Observe."

### Verification
How do we know the plan is safe?
*   **Pre-conditions**: Can I pick up the cup? (Only if hand is empty).
*   **Post-conditions**: Did I pick up the cup? (Check gripper sensor).

### Industry Vocab
*   **STRIPS**: Stanford Research Institute Problem Solver (classic planning).
*   **PDDL**: Planning Domain Definition Language.
*   **ReAct**: Reason + Act prompting strategy.

### Code Example: Plan Execution
```python
# Defensive Plan Executor
def execute_plan(plan_json):
    for step in plan_json['steps']:
        action = step['action']
        if action == 'navigate':
            # Check battery before long move
            if battery_level < 10:
                return "Aborted: Low Battery"
            result = nav_client.send_goal(step['target'])
        elif action == 'pick':
            result = gripper_client.send_goal(step['object'])
        
        if not result.success:
            return f"Failed at step: {action}"
    return "Success"
```

## Real-World Use Case: The "Bring me" Task
"Bring me the red screwdriver." The robot must:
1.  Map "red screwdriver" to a visual target.
2.  Search the room (Planning: where are tools usually kept?).
3.  Navigate to the table.
4.  Pick it up.
5.  Return to user.
The LLM orchestrates this entire sequence.