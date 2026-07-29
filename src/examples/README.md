# SemaFORR examples

This data-only ROS 2 package installs the inherited simulator maps, navigation
graphs, and target/configuration fixtures under
`share/semaforr_examples/core`.

Nodes and launch files should resolve assets through the ament package index;
they must not depend on this source-tree location. The files retain their
legacy formats so existing experiments remain reproducible while individual
scenarios are converted to the validated SemaFORR YAML configuration.
