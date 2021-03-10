
NAME=$1
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
rosservice call /world/build_octomap "{bounding_box_origin: {x: -0.0, y: -0.0, z: 0.0}, bounding_box_lengths: {x: 30, y: 30, z: 15}, leaf_size: 0.2, filename: '$DIR/octomaps/$NAME.bt'}" --wait
