# Attribution and publication scope

This repository contains a single-gauge horizontal image-alignment demonstration: rule-teacher behavior cloning for joint 1, capture-quality checks, HOG/linear-SVM classification, and fixed home return. Pose-selection reinforcement learning and multi-target coordination are outside the completed scope.

Root LICENSE is Apache-2.0. Camera inspection code declares Apache-2.0. CRANE+ source package metadata declares Apache License 2.0; its license text is retained under THIRD_PARTY_LICENSES. The generated robot URDF is derived from that description.

Full CRANE meshes, ROS 2, MoveIt 2, Ignition/Gazebo, the Docker image and Japanese fonts are external dependencies and are not redistributed. Their respective licenses and attribution requirements continue to apply. Simulated gauge primitives and recorded gauge images are supplied; no external stock texture or generated-AI photograph is used as camera evidence.

NOTICE_ORIGIN.md preserves historical source provenance. Its earlier runtime/domain descriptions are historical. The active-inspection controller uses ROS domain 47 and partition physicalai_active_47. The classifier's pinned camera_demo runtime configuration describes its training/preprocessing environment and must not be used to silently change the active controller's communication domain.

multigauge/ contains reused transport/geometry and image-classification helpers. This repository does not present a completed multi-gauge coordinator. Recorded normal and abnormal video examples use the same image classifier but different control-model configurations; the normal example did not invoke a correction policy.

The PDF in docs/ is the current explanation supplied for this publication. Original work files and earlier deliverables remain outside this new repository. PUBLIC_FILE_PROVENANCE.json records source-relative paths and hashes for the published copies and authored documentation.
