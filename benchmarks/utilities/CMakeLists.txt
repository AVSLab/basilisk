add_executable(benchmark_eigen_linear_algebra EXCLUDE_FROM_ALL benchmark_eigen_linear_algebra.cpp)
target_link_libraries(benchmark_eigen_linear_algebra ArchitectureUtilities)
set_target_properties(benchmark_eigen_linear_algebra PROPERTIES FOLDER "benchmarks/utilities")

add_custom_target(benchmark_eigen_linear_algebra_smoke
    COMMAND benchmark_eigen_linear_algebra --smoke
    DEPENDS benchmark_eigen_linear_algebra
    COMMENT "Running Eigen versus linearAlgebra benchmark smoke test"
    VERBATIM
)
add_dependencies(benchmark_smoke_tests benchmark_eigen_linear_algebra_smoke)
