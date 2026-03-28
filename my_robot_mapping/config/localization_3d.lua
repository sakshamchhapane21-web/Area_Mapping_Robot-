include "mapping.lua"

-- Pure localization mode: Do not build new submaps, just match against the loaded ones
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Fast optimization since we aren't changing the global map
POSE_GRAPH.optimize_every_n_nodes = 20

return options
