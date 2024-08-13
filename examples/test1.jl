
begin
    ##### an example for using PID with limited number of integrals ######################################################################
    using PIDSmoothing
    using Plots
    using Random

    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #y=Float32.([(rand(10)*5).+20; rand(100).*2; (rand(50).*5).-50; rand(70).-10; (rand(25).*5).-40; rand(20).*5])
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit = 3.0
    integral_length = 10
    adaptive_rate=0.01
    decay=0.2
    filtered = pid_smoothing(y, n_setpoint=2,  ki=ki, kp=kp, kd=kd,
                    adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                    integral_length=integral_length, decay=decay,
                    neighbors_before=true, neighbors_after=true)
    plot(filtered, lw=2) #, 0.1, 0.01, 0.01, 10
    #plot!(setpoints)
    plot!(y)
end

begin
    #### an example with only limited value of integral ######################################################################################
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #y=[rand(10).+2; rand(10).-2; rand(5).+5; rand(20).+1; rand(25).-4; rand(20).*4]
    
    plot(pid_smoothing(y, n_setpoint=5, integral_limit=1.0, decay=0.5,
        adaptive_rate=0.01,# integral_length=20,
         neighbors_before=false, neighbors_after=true), lw=2) #, 0.1, 0.01, 0.01, 10
    plot!(y)
end

begin
    #### an example of batch smoothing with only limited value of integral ##################################################################
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y1 = sin.(x) .+ randn(length(x)) .* 0.1
    y2 = cos.(x) .+ randn(length(x)) .* 0.1
    #y3 = sin.(x).^2 .+ randn(length(x)) .* 0.1
    y=[[y1];[y2]]#;[y3]]
    plot(batch_pid_smoothing([Float32.(vec) for vec in y], n_setpoint=5, integral_limit=2.0, 
                                          adaptive_rate=0.1,
                                          neighbors_before=true, neighbors_after=true), lw=2) #, 0.1, 0.01, 0.01, 10
    plot!(y)
end

begin
    #### an example of batch smoothing with limited number of integrals ############################################################
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y1 = sin.(x) .+ randn(length(x)) .* 0.1
    y2 = cos.(x) .+ randn(length(x)) .* 0.1
    #y3 = sin.(x).^2 .+ randn(length(x)) .* 0.1
    y=[[y1];[y2]]
    plot(batch_pid_smoothing([Float32.(vec) for vec in y], n_setpoint=5, decay=0.2,
                                           integral_length=10, integral_limit=2.0, adaptive_rate=0.001,
                                           neighbors_before=false, neighbors_after=true), lw=2) #, 0.1, 0.01, 0.01, 10
    plot!(y)
end

begin
    ### an example of weighted integral window ##################################################################################
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #integral_length=10
    weights=Float64.(collect(0:(1/9):1))
    #weights=reverse(weights)
    plot(weighted_pid_smoothing(Float64.(y), decay=0.2,
                                             weights=weights.^1, n_setpoint=4, adaptive_rate=0.01,
                                             neighbors_before=true, neighbors_after=true, 
                                             integral_limit=1.0), lw=2) 
                                          
    plot!(y)
end


begin
    ### an example of realtime_pid_smoothing ###############################################################################
    using PIDSmoothing
    using Plots
    using PIDSmoothing

    # Initialize PID controller with default or custom parameters
    kp = 0.2
    ki = 0.001
    kd = 0.1
    integral_limit =1.0# Inf
    integral_length=10
    adaptive_rate=0.01
    pid = PIDSmoothing.NumberLimitPID(kp, ki, kd, integral_length, adaptive_rate, integral_limit)
    #pid = PIDSmoothing.ValueLimitPID(kp, ki, kd, integral_limit, adaptive_rate)
    

   
    x = range(0, stop=20, length=200)
    data_stream = Float64.(sin.(x) .+ randn(length(x)) .* 0.1)
    data_smoothed=zeros(Float64, length(data_stream))
    data_smoothed[1]=data_stream[1]


    # Initialize the first smoothed value
    previous_smoothed_value = data_stream[1]
    n_setpoint=3
    # Process the data stream
    for i in 2:length(data_stream)
        new_data_points = data_stream[max(1, i-n_setpoint):i]

        #setpoint = mean(data_stream[max(1, i-DEFAULT_SETPOINT_NUMBERS+1):i])
        smoothed_value = realtime_pid_smoothing(pid, new_data_points, previous_smoothed_value, decay=0.1)
        
        # Update the previous smoothed value
        previous_smoothed_value = smoothed_value

        # Print or store the smoothed value
        println("Smoothed value at step $i: $smoothed_value")
        data_smoothed[i]=smoothed_value
    end
    plot(data_smoothed, lw=2)
    plot!(data_stream)
end


##############################################  comparisions  ##########################################
begin
    ####################################### example 1 ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess

    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #y=Float32.([(rand(10)*5).+20; rand(100).*2; (rand(50).*10).+5; rand(70).-10; (rand(25).*20).-40; rand(20).*4])
    kp = 0.1
    ki = 0.05
    kd = 0.2
    integral_limit =2.0
    integral_length=10
    adaptive_rate=0.01
    plot(pid_smoothing(y,  n_setpoint=5, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length,
         neighbors_before=true, neighbors_after=true), label="PID",
         alpha=0.8, lw=2) #
    
    
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.1)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.8)
    plot!(y, label="Noisy Data", alpha=0.5 )
end

begin
    ####################################### example 2 ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess

    Random.seed!(0)
    x = range(0, stop=30, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.4
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =10.0
    integral_length=10
    adaptive_rate=0.01
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=5, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length, decay=0.2,
         neighbors_before=true, neighbors_after=true), label="PID",
         alpha=0.8, lw=2) #
    
    
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.1)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.8)
    plot!(y, label="Noisy Data", alpha=0.5 )
end

###############################################################################################################################
begin
    ####################################### example 3 ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess
    using SavitzkyGolay
    Random.seed!(0)
    # Generating the signal
    t = 0:0.02:10
    signal = sin.(2π.*t) .+ 0.5*rand(length(t))  # Sinusoidal with Gaussian noise
    outliers = [rand() < 0.01 ? 2*randn() : 0 for _ in t]  # Random outliers
    y=noisy_signal = signal .+ outliers
    
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =2.0
    integral_length=10
    adaptive_rate=0.01
    n_setpoint=5
    decay=0.2
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length, decay=decay,
         neighbors_before=true, neighbors_after=true), label="PID",
         alpha=0.8, lw=2) #
   
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.1)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.8)

    plot!(noisy_signal, alpha=0.5, label="Noisy data" )
end

########################################################################################################################################
begin
    ####################################### example 4 (step signal) ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess
    using SavitzkyGolay

    Random.seed!(150)
    # Generate a step function signal
    function generate_step_function(t)
        signal = zeros(length(t))
        for i in eachindex(t)
            if t[i] < 2
                signal[i] = 1
            elseif t[i] < 4
                signal[i] = 3
            elseif t[i] < 6
                signal[i] = -2
            else
                signal[i] = 2
            end
        end
        return signal
    end

    # Generate time vector and step function signal
    t = 0:0.01:8
    step_signal = generate_step_function(t)

    # Add Gaussian noise
    noise = 0.5 * randn(length(t))
    noisy_step_signal = step_signal .+ noise

    
    y=noisy_step_signal
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =1.0
    integral_length=10
    adaptive_rate=0.0001
    n_setpoint=15
    decay=0.2
    plot(pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length, decay=decay,
         neighbors_before=true, neighbors_after=true), label="PID",
         alpha=0.8, lw=3) #
   
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.2)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.5)

    sg = savitzky_golay(y, 51, 4)
    sg_smoothed=sg.y
    plot!(sg_smoothed, alpha=0.5, label="SavitzkyGolay", lw=2)

    plot!(noisy_step_signal, alpha=0.2)
end

#############################################################################################################################################
begin
    ####################################### example 5 (Piecewise Linear with Peaks) ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess
    using SavitzkyGolay
    Random.seed!(10)
    # Define the time vector
    t = 0:0.1:200

    # Add Gaussian noise
    noise = 0.5 * randn(length(t))
   


    # Piecewise Linear with Random Peaks
    linear_signal = [i < 50 ? 0.1*i : (i < 100 ? 0.1*(i-50) : (i < 150 ? 0.1*(i-100) : 0.1*(i-150))) for i in t]
    linear_signal[51] += 5
    linear_signal[101] += -3
    linear_signal[151] += 10
    linear_signal .+= noise



    y=linear_signal
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =1.0
    integral_length=10
    adaptive_rate=0.0001
    n_setpoint=25
    decay=0.3
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
        adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
        integral_length=integral_length, decay=decay,
        neighbors_before=true, neighbors_after=true), label="PID",
        alpha=0.8, lw=3) #

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.1)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.5)

    sg = savitzky_golay(y, 81, 4)
    sg_smoothed=sg.y
    plot!(sg_smoothed, alpha=0.5, label="SavitzkyGolay", lw=2)

    plot!(linear_signal, label="Piecewise Linear with Peaks", alpha=0.2)
        
end

#############################################################################################################################
begin
    ####################################### example 6 (exp_decay_signal) ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess
    using SavitzkyGolay

    Random.seed!(20)
    
    # Define the time vector
    t = 0:0.1:200

    # Add Gaussian noise
    noise = 0.1 * randn(length((t)))
    # Exponential Decay with Oscillations
    exp_decay_signal = exp.(-0.01 .* t) .* sin.(2 .* pi .* t ./ 20) #./sqrt.(t)

    y=exp_decay_signal.+ noise
    kp = 0.1
    ki = 0.01
    kd = 0.1
    integral_limit =2.0
    integral_length=20
    adaptive_rate=0.01
    n_setpoint=12
    decay=0.3
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=n_setpoint, 
        ki=ki, kp=kp, kd=kd, decay=decay,
        adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
        integral_length=integral_length,
        neighbors_before=true, neighbors_after=true), label="PID",
        alpha=0.8, lw=2) #

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.05)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.8)


    plot!(y, label="Exponential Decay with Oscillations", alpha=0.2, lw=1)
    
end

##################################################################################################################################
begin
    ####################################### example 7 (Noisy Signal with Steps & Nonlinearity) ##############################
    using PIDSmoothing
    using Random
    using Plots
    using Loess
    Random.seed!(10)
    # Generate a time vector
    t = LinRange(0, 10, 1000)

    # Baseline trend (slow sinusoidal)
    baseline = sin.(2π * 0.1 * t)

    # High-frequency noise
    noise = 0.1 * randn(length(t))

    # Sudden steps
    steps = zeros(length(t))
    steps[200:end] .= 1.0
    steps[500:end] .= -1.5

    # Nonlinear trend
    nonlinear_trend = 0.1 * t.^2

    # Combine all components into the final signal
    signal = baseline + steps + nonlinear_trend

    # (Optional) Plot the signal to visualize
    #plot(signal, label="Noisy Signal with Steps & Nonlinearity")
    y=signal + noise
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =2.0
    integral_length=10
    adaptive_rate=0.01
    n_setpoint=10
    decay=0.1
    PIDSmoothed=pid_smoothing(y ,n_setpoint=n_setpoint, 
                ki=ki, kp=kp, kd=kd, decay=decay,
                adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                integral_length=integral_length,
                neighbors_before=true, neighbors_after=true)
    plot(PIDSmoothed, lw=3, alpha=0.8, label="PID")

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.1)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.5)
    #plot!(signal, label="main signal")
    plot!(y, label="noisy_signal", alpha=0.3)
    #sum(abs.(PIDSmoothed.-signal))-sum(abs.(vs.-signal))
    
end

###################################################################################################################################################
begin
    ####################################### example 7 (synthetic stock price signal) ############################## 
    using Plots
    using PIDSmoothing
    using Random
    using Plots
    using Loess

    Random.seed!(850)
    # Function to generate a synthetic stock price signal
    function generate_stock_price_signal(N::Int; init_price::Float64 = 100.0, volatility::Float64 = 0.02, trend::Float64 = 0.001)
        prices = Vector{Float64}(undef, N)
        prices[1] = init_price
        
        for i in 2:N
            # Random noise simulating daily price fluctuation
            noise = volatility * randn()
            # Trend component (upwards or downwards)
            prices[i] = prices[i-1] * (1.0 + trend + noise)
        end
        
        return prices
    end

    # Parameters for the signal
    N = 2000  # Length of the signal
    init_price = 100.0  # Initial stock price
    volatility = 0.025  # Volatility factor
    trend = 0.001  # Overall trend

    # Generate the signal
    stock_price_signal = generate_stock_price_signal(N; init_price=init_price, volatility=volatility, trend=trend)

    # Plot the generated stock price signal
    plot(stock_price_signal, title="Simulated Stock Price Signal", xlabel="Time", ylabel="Price", lw=2)

    y=stock_price_signal
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =2.0
    integral_length=10
    adaptive_rate=0.001
    n_setpoint=10
    decay=0.2
    PIDSmoothed=pid_smoothing(y ,n_setpoint=n_setpoint, 
                ki=ki, kp=kp, kd=kd, decay=decay,
                adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                integral_length=integral_length,
                neighbors_before=true, neighbors_after=true)
    start=1
    finish=2000
    plot(PIDSmoothed[start:finish], lw=2, alpha=0.8, label="PID")

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=0.05)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs[start:finish], lw=1, label="Loess", alpha=0.5)
    
    plot!(y[start:finish], label="noisy_signal", alpha=0.3)
    

end

