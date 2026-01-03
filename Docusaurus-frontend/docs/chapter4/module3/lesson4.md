---
id: lesson4
title: Nav2 Path planning for bipedal humanoid movement
sidebar_label: Sim-to-real transfer techniques
---

# Nav2: Path planning for bipedal humanoid movement.

## Heading Breakdown
**Nav2: Path planning for bipedal humanoid movement** adapts the standard navigation stack for walking robots. **Nav2** is the second generation of the ROS Navigation stack, built on behavior trees. **Path planning** is the calculation of a trajectory (list of points) to a goal. **Bipedal humanoid movement** adds a layer of complexity: unlike a wheeled robot, a humanoid can step over small obstacles but cannot turn in place instantly without stepping. The importance is **mobility**; a robot that can see but cannot move is a statue. Real usage involves tuning the **DWB (Dynamic Window Approach)** controller to generate velocity commands that match the **Unitree G1**'s walking gait frequency. An example is setting a "stair cost" in the costmap so the robot prefers the ramp but will take the stairs if necessary. This is key for **upgradable systems** where we might upgrade the legs to be more agile, requiring a retuning of the planner.

*(Note: Sidebar refers to Sim-to-Real, but per mapping, we cover Nav2 here).*

## Training Focus: Navigation Semantics
We focus on **safety**.
*   **Costmaps**: Layers of danger (inflation layer, obstacle layer).
*   **Recovery Behaviors**: What to do when stuck (e.g., "backup", "spin").

## Detailed Content
### The Nav2 Architecture
*   **Planner Server**: Global path (A*, Dijkstra).
*   **Controller Server**: Local path following (DWB, MPC).
*   **Behavior Tree**: The logic (Sequence, Fallback).

### Bipedal Constraints
*   **Footprint**: A humanoid's footprint changes as it walks. We approximate it with a radius.
*   **Sway**: Humanoids sway side-to-side. The planner must account for this to avoid hitting doorframes.

### Industry Vocab
*   **Holonomic**: Can move in any direction (humanoids are pseudo-holonomic).
*   **Voxel Grid**: 3D representation of obstacles.
*   **BT (Behavior Tree)**: XML logic flow.

### Code Example: Custom Behavior Tree
```xml
<!-- Defensive Behavior Tree -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
        <Wait wait_duration="5"/> <!-- Wait for balance to stabilize -->
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Real-World Use Case: Crowds
Navigating a G1 through a crowded hallway. We use the **Social Costmap Layer** to create "repulsion zones" around people, ensuring the robot doesn't invade personal space while squeezing through the crowd.