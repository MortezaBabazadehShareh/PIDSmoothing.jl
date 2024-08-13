# runtests.jl

using Test
using PIDSmoothing

# Test for NumberLimitPID constructor
@testset "NumberLimitPID Tests" begin
    pid = NumberLimitPID(0.1, 0.01, 0.01, 10, 0.0, 10.0)
    @test pid.kp == 0.1
    @test pid.ki == 0.01
    @test pid.kd == 0.01
    @test pid.integral_length == 10
    @test pid.integral_limit == 10.0
end

# Test for ValueLimitPID constructor
@testset "ValueLimitPID Tests" begin
    pid = ValueLimitPID(0.1, 0.01, 0.01, 10.0, 0.0)
    @test pid.kp == 0.1
    @test pid.ki == 0.01
    @test pid.kd == 0.01
    @test pid.integral_limit == 10.0
end

# Test for update! with NumberLimitPID
@testset "Update! Tests with NumberLimitPID" begin
    pid = NumberLimitPID(0.1, 0.01, 0.01, 10, 0.0, 10.0)
    output = PIDSmoothing.update!(pid, 2.0, 5.0)
    @test typeof(output) <: AbstractFloat
end

# Test for update! with ValueLimitPID
@testset "Update! Tests with ValueLimitPID" begin
    pid = ValueLimitPID(0.1, 0.01, 0.01, 10.0, 0.0)
    output = PIDSmoothing.update!(pid, 2.0, 5.0)
    @test typeof(output) <: AbstractFloat
end

# Test for pid_smoothing
@testset "pid_smoothing Tests" begin
    data = [1.0, 2.5, 1.4, 3.5, 3.0, 4.2, 5.0, 6.1, 6.8, 8.1]
    smoothed_data = pid_smoothing(data, kp=0.1, ki=0.01, kd=0.01, n_setpoint=2, integral_limit=2.0, integral_length=3)
    @test length(smoothed_data) == length(data)
    @test all(x -> x >= 1.0, smoothed_data)  # example check
end

# Test for batch_pid_smoothing
@testset "batch_pid_smoothing Tests" begin
    data_batch = [[1.0, 2.0, 3.0, 4.0], [10.0, 20.0, 15.0, 10.0]]
    smoothed_batch = batch_pid_smoothing(data_batch)
    @test length(smoothed_batch) == length(data_batch)
    @test all(x -> length(x) == 4, smoothed_batch)
end

# Test for weighted_pid_smoothing
@testset "weighted_pid_smoothing Tests" begin
    data = [1.0, 2.5, 1.4, 3.5, 3.0, 4.2, 5.0, 6.1, 6.8, 8.1]
    weights = Float64.(collect(0:(1/3):1))
    smoothed_data = weighted_pid_smoothing(data, weights=weights)
    @test length(smoothed_data) == length(data)
    @test all(x -> x >= 1.0, smoothed_data)  # example check
end

# Test for realtime_pid_smoothing
@testset "realtime_pid_smoothing Tests" begin
    pid = NumberLimitPID(0.2, 0.001, 0.1, 10, 0.0002, 1.0)
    previous_smoothed_value = 18.0
    new_data_points = [17.0, 19.0, 20.0]
    smoothed_value = realtime_pid_smoothing(pid, new_data_points, previous_smoothed_value)
    @test typeof(smoothed_value) <: AbstractFloat
end
