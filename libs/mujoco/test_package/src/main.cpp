#include <mujoco/mujoco.h>

// Just checking that we can actually run a mujoco function
int main(int argc, char** argv) {
  auto res = mju_min(0, 1);
  return res == 0 ? 0 : 1;
}
