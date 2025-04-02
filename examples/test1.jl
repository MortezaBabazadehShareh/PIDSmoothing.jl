
begin
    ##### an example for using PID with limited number of integrals ######################################################################
    using Revise
    using PIDSmoothing
    using Plots
    using Random, Loess
    #using LinearAlgebra

    Random.seed!(20)
    x = range(0, stop=20, length=200)
    y_real = sin.(x) 
    y= y_real .+ randn(length(x)) .* 0.1 # .* collect(1:length(x)/1)./20#; reverse(collect(1:length(x)/2))./20]
    #y=Float32.([(rand(10)*5).+20; rand(100).*2; (rand(50).*5).-50; rand(70).-10; (rand(25).*5).-40; rand(20).*5])
    kp = 0.3
    ki = 0.2
    kd = 0.1
    integral_limit = 0.5
    integral_length = 10
    adaptive_rate=0.35
    decay=0.35
    n_setpoint=15
    filtered = pid_smoothing(y, n_setpoint=n_setpoint,  ki=ki, kp=kp, kd=kd,
                    adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                    integral_length=integral_length, decay=decay,
                    neighbors_before=true, neighbors_after=true)
    plot(filtered, lw=2, label="Smoothed", xlabel="Time", ylabel="Value") #, 0.1, 0.01, 0.01, 10
    #scatter!(setpoints, ms = 1)
    
    plot!(y_real, label="Ground Truth", alpha=0.8)
    
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span = n_setpoint/length(y)*2)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.8)
    
    #savefig("C:\\Users\\babazadehmorteza\\GroupMeeting2025fig16.svg")
end

begin
    #### an example with only limited value of integral ######################################################################################
    using Revise
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=400)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #y=[rand(10).+2; rand(10).-2; rand(5).+5; rand(20).+1; rand(25).-4; rand(20).*4]
    
    plot(pid_smoothing(y, n_setpoint= 25, integral_limit=1.0, decay=0.1,
         adaptive_rate=0.5, ki=0.1, kp=0.5, kd=0.01,# integral_length=20,
         neighbors_before=true, neighbors_after=true), lw=2) #, 0.1, 0.01, 0.01, 10
    plot!(y)
end

begin
    #### an example of batch smoothing with only limited value of integral ##################################################################
    using Revise
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=400)
    y1 = sin.(x) .+ randn(length(x)) .* 0.1
    y2 = cos.(x) .+ randn(length(x)) * 0.1
    #y3 = sin.(x).^2 .+ randn(length(x)) .* 0.1
    y=[[y1];[y2]]#;[y3]]
    plot(batch_pid_smoothing([Float32.(vec) for vec in y], n_setpoint=20, integral_limit=0.5,
                                          adaptive_rate=0.25,  ki=0.2, kp=0.3, kd=0.1,
                                          neighbors_before=true, neighbors_after=true), lw=2) #, 0.1, 0.01, 0.01, 10
    plot!(y)
end

begin
    #### an example of batch smoothing with limited number of integrals ############################################################
    using Revise
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y1 = sin.(x) .+ randn(length(x)) .* 0.1
    y2 = cos.(x) .+ randn(length(x)) .* 0.1
    #y3 = sin.(x).^2 .+ randn(length(x)) .* 0.1
    y=[[y1];[y2]]
    plot(batch_pid_smoothing([Float64.(vec) for vec in y], n_setpoint=25, decay=0.1,  ki=0.2, kp=0.3, kd=0.1,
                                           integral_length=15, integral_limit=0.3, adaptive_rate=0.25,
                                           neighbors_before=true, neighbors_after=true), lw=2) #, 0.1, 0.01, 0.01, 10
    plot!(y)
end

begin
    ### an example of weighted integral window ##################################################################################
    using Revise
    using PIDSmoothing
    using Plots
    using Random


    Random.seed!(20)
    x = range(0, stop=20, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #integral_length=10
    weights=Float64.(collect(0:(1/9):1.0))
    #weights=reverse(weights)
    plot(weighted_pid_smoothing(Float64.(y), decay=0.15,
                                             weights=weights.^1, n_setpoint=25, adaptive_rate=0.15,
                                             neighbors_before=true, neighbors_after=true,  ki=0.3, kp=0.3, kd=0.1, 
                                             integral_limit=1.0), lw=2) 
                                          
    plot!(y)
end


begin
    ### an example of realtime_pid_smoothing ###############################################################################
    using PIDSmoothing
    using Plots
    using PIDSmoothing
    Random.seed!(0)
    # Initialize PID controller with default or custom parameters
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit =0.5# Inf
    integral_length=10
    adaptive_rate=0.5
    pid = PIDSmoothing.NumberLimitPID(kp, ki, kd, integral_length, adaptive_rate, integral_limit)
    #pid = PIDSmoothing.ValueLimitPID(kp, ki, kd, integral_limit, adaptive_rate)
    

   
    x = range(0, stop=20, length=200)
    data_stream = Float64.(sin.(x) .+ randn(length(x)) .* 0.1)
    data_smoothed=zeros(Float64, length(data_stream))
    data_smoothed[1]=data_stream[1]


    # Initialize the first smoothed value
    previous_smoothed_value = data_stream[1]
    n_setpoint=20
    # Process the data stream
    for i in 2:length(data_stream)
        #
        new_data_points = data_stream[max(1, i-n_setpoint):i]
        #new_data_points = [data_smoothed[max(1, i-n_setpoint):i-1]..., data_stream[i]...]
        #println("new_data_points: $new_data_points")
        #setpoint = mean(data_stream[max(1, i-DEFAULT_SETPOINT_NUMBERS+1):i])
        smoothed_value = realtime_pid_smoothing(pid, new_data_points, previous_smoothed_value)
        
        # Update the previous smoothed value
        previous_smoothed_value = smoothed_value

        # Print or store the smoothed value
        #println("Smoothed value at step $i: $smoothed_value")
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
    kp = 0.3
    ki = 0.1
    kd = 0.1
    integral_limit =0.5
    integral_length=10
    adaptive_rate=0.25
    n_setpoint=30
    plot(pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length,
         neighbors_before=true, neighbors_after=true), label="PID",
         alpha=0.8, lw=2) #
    
    
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span = n_setpoint/length(y)*2)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.8)
    plot!(y, label="Noisy Data", alpha=0.5 )
end

begin
    ####################################### example 2 ##############################
    using Revise
    using PIDSmoothing
    using Plots
    using Random
    using Loess

    Random.seed!(0)
    x = range(0, stop=30, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.2
    kp = 0.3
    ki = 0.1
    kd = 0.1
    integral_limit =1.0
    integral_length=10
    adaptive_rate=0.2
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=30, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length, decay=0.2,
         neighbors_before=true, neighbors_after = true), label="PID",
         alpha=0.8, lw=2) #
    
    
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span = n_setpoint/length(y)*2)# n_setpoint/length(y)*2)
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
    
    kp = 0.4
    ki = 0.1
    kd = 0.01
    integral_limit =1.0
    integral_length=10
    adaptive_rate=0.2
    n_setpoint=25
    decay=0.2
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length, decay=decay,
         neighbors_before=true, neighbors_after=true), label="PID",
         alpha=0.8, lw=2) #
   
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span = n_setpoint/length(y)*2)
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
    plot(noisy_step_signal, alpha=0.3, label="Noisy data")

    
    y=noisy_step_signal
    kp = 0.1
    ki = 0.1
    kd = 0.01
    integral_limit = 0.5
    integral_length=10
    adaptive_rate=0.25
    n_setpoint=50
    decay=0.25
    plot!(pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
         adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
         integral_length=integral_length, decay=decay,
         neighbors_before=true, neighbors_after=true), label="PID Smoothing",
         alpha=0.8, lw=2, xlabel="Time", ylabel="Value") #
   
    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span= n_setpoint/length(y)*2)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.5)

    #sg = savitzky_golay(y, 51, 4)
    #sg_smoothed=sg.y
    #plot!(sg_smoothed, alpha=0.5, label="SavitzkyGolay", lw=2, leg=:topright)

end

#############################################################################################################################################
begin
    ####################################### example 5 (Piecewise Linear with Peaks) ##############################
    using PIDSmoothing
    using Plots
    using Random
    using Loess
    using SavitzkyGolay
    Random.seed!(25)
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
    kp = 0.3
    ki = 0.01
    kd = 0.01
    integral_limit =0.5
    integral_length=10
    adaptive_rate=0.25
    n_setpoint=50
    decay=0.15
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=n_setpoint, ki=ki, kp=kp, kd=kd,
        adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
        integral_length=integral_length, decay=decay,
        neighbors_before=true, neighbors_after=true), label="Smoothed",
        alpha=1.0, lw=3) #

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span=n_setpoint/length(y)*2)
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us)
    plot!(vs, lw=2, label="Loess", alpha=0.5)

    sg = savitzky_golay(y, 81, 4)
    sg_smoothed=sg.y
    #plot!(sg_smoothed, alpha=0.5, label="SavitzkyGolay", lw=2)

    plot!(linear_signal, label="Piecewise Linear with Peaks", alpha=0.4)
        
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
    kp = 0.3
    ki = 0.2
    kd = 0.1
    integral_limit =0.3
    integral_length=10
    adaptive_rate=0.25
    n_setpoint=100
    decay=0.25
    plot(PIDSmoothing.pid_smoothing(y,  n_setpoint=n_setpoint, 
        ki=ki, kp=kp, kd=kd, decay=decay,
        adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
        integral_length=integral_length,
        neighbors_before=true, neighbors_after=true), label="PID",
        alpha=0.8, lw=2) #

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span= n_setpoint/length(y)*2)
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

    
    y = signal + noise
    kp = 0.3
    ki = 0.3
    kd = 0.1
    integral_limit =0.5
    integral_length=10
    adaptive_rate = 0.2
    n_setpoint=40
    decay=0.25
    PIDSmoothed=pid_smoothing(y ,n_setpoint=n_setpoint, 
                ki=ki, kp=kp, kd=kd, decay=decay,
                adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                integral_length=integral_length,
                neighbors_before=true, neighbors_after=true)
    plot(PIDSmoothed, lw=2, alpha=0.8, label="PID")

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span = n_setpoint/length(y)*2)
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
    using Revise
    using Plots
    using PIDSmoothing
    using Random
    using Plots
    using Loess

    Random.seed!(5)
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
    N = 500  # Length of the signal
    init_price = 100.0  # Initial stock price
    volatility = 0.025  # Volatility factor
    trend = 0.001  # Overall trend

    # Generate the signal
    stock_price_signal = generate_stock_price_signal(N; init_price=init_price, volatility=volatility, trend=trend)

    ##smoothing
    y = stock_price_signal
    kp = 0.3
    ki = 0.1
    kd = 0.1
    integral_limit = 1.0
    integral_length = 10
    adaptive_rate=0.25
    n_setpoint=50
    decay=0.25
    PIDSmoothed=pid_smoothing(y ,n_setpoint = n_setpoint, 
                ki=ki, kp=kp, kd=kd, decay=decay,
                adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                integral_length=integral_length,
                neighbors_before=true, neighbors_after=true)
    start=1
    finish=length(y)
    plot(y[start:finish], label="noisy_signal", alpha=0.4, xlabel="Time", ylabel="Value")       
    plot!(PIDSmoothed[start:finish], lw=2, alpha=1.0, label="PID Smoothing")

    xs=Float64.(range(1, length(y), length(y)))
    model=loess(xs, Float64.(y), span =  min(length(y)/2, n_setpoint/length(y)*2))
    us = range(extrema(xs)...; step = 1)
    vs = predict(model, us) ### vs is the smoothed data using Loess
    plot!(vs[start:finish], lw=2, label="Loess", alpha=0.5)
    #savefig("C:\\Users\\babazadehmorteza\\PIDPackageTest1.png")

end

#### derivative similarity as a metric
begin
    function local_trend_direction_agreement_window(original::Vector, smoothed::Vector; k::Int=1)
        @assert length(original) == length(smoothed) "Signals must be the same length"
        N = length(original)
        @assert k > 0 && k < N "Window size k must be between 1 and N-1"
    
        match_count = 0
        total = N - k
    
        for t in 1:total
            d_orig = original[t + k] - original[t]
            d_smooth = smoothed[t + k] - smoothed[t]
    
            if sign(d_orig) == sign(d_smooth)
                match_count += 1
            end
        end
    
        return match_count / total  # Returns ratio of matching directions
    end
    noisy_signal = y #noisy signal
    pid_smooth = PIDSmoothed #smoothed signal using PID
    loess_smooth = vs #smoothed signal using Loess

    # Compare PID and LOESS at different window sizes
    for k in (1, 3, 5, 7, 9)
        ltda_pid = local_trend_direction_agreement_window(noisy_signal, pid_smooth, k=k)
        ltda_loess = local_trend_direction_agreement_window(noisy_signal, loess_smooth, k=k)
        
        println("Window k=$k")
        println("  PID LTDA:   ", round(ltda_pid * 100, digits=2), "%")
        println("  LOESS LTDA: ", round(ltda_loess * 100, digits=2), "%")
    end
end

### local curve similarity as a metric
begin
    using Polynomials
    function window_curve_similarity(noisy::Vector, smoothed::Vector; window=11, degree=2)
        @assert length(noisy) == length(smoothed)
        N = length(noisy)
        half_w = div(window, 2)
        similarities = Float64[]
    
        for t in (half_w+1):(N - half_w)
            # Define local window
            idx = (t - half_w):(t + half_w)
            x = collect(1:window)
            y = noisy[idx]
    
            # Fit polynomial
            p = fit(x, y, degree)
    
            # Evaluate fitted curve
            y_fit = p.(x)
    
            # Compare to smoothed signal in the same window
            y_smooth = smoothed[idx]
            sim = cor(y_fit, y_smooth)  # Could also use 1 - MSE
    
            push!(similarities, sim)
        end
    
        return mean(similarities)
    end
    
    noisy_signal = y #noisy signal
    pid_smooth = PIDSmoothed #smoothed signal using PID
    loess_smooth = vs #smoothed signal using Loess

    windows = 11
    Wcs_pid = window_curve_similarity(noisy_signal, pid_smooth, window = windows)
    Wcs_loess = window_curve_similarity(noisy_signal, loess_smooth, window = windows)
    
    #println("Window k=$k")
    println("  PID WTDA:   ", round(Wcs_pid * 100, digits=2), "%")
    println("  LOESS WTDA: ", round(Wcs_loess * 100, digits=2), "%")

end

#### lag as metric
begin  
    using DSP
    signal = y
    # Define the lag range: from -(N-1) to (N-1), where N is the length of the input signals
    N = length(signal)
    lags = -N+1:N-1


    cross_corrolation_PID = xcorr(signal, PIDSmoothed)
    max_index_PID = argmax(cross_corrolation_PID)
    lag_PID = lags[max_index_PID]
    print("Lag PID =  \t $lag_PID \n")

    cross_corrolation_LOESS = xcorr(signal, vs)
    max_index_LOESS = argmax(cross_corrolation_LOESS)
    lag_LOESS = lags[max_index_LOESS]
    print("Lag LOESS =  \t $lag_LOESS \n")

    #cross_corrolation_SG = xcorr(signal, sg_smoothed)
    #max_index_SG = argmax(cross_corrolation_SG)
    #lag_SG = lags[max_index_SG]
    #print("Lag SG =  \t $lag_SG \n")
end

### correlation as a metric
begin 
    using Statistics
    signal = y
    correlation_coefficient_PID = cor(signal, PIDSmoothed)
    print("correlation coefficient PID \t =  \t $correlation_coefficient_PID \n")

    correlation_coefficient_LOESS = cor(signal, vs)
    print("correlation coefficient LOESS \t =  \t $correlation_coefficient_LOESS \n")

    #correlation_coefficient_SG = cor(step_signal, sg_smoothed)
    #print("correlation coefficient SG \t =  \t $correlation_coefficient_SG \n")

end
### Total Variation as metric
begin 
    function calculate_tv(signal::Vector{Float64})::Float64
        # Ensure the signal has at least two elements
        if length(signal) < 2
            throw(ArgumentError("Signal must have at least two elements."))
        end
        
        # Calculate the total variation
        total_variation = sum(abs.(diff(signal)))
        return total_variation
    end
    
    # Example usage
    
    signal = y
    # Calculate Total Variation
    tv_step = calculate_tv(signal)
    tv_pid = calculate_tv(PIDSmoothed)
    tv_loess = calculate_tv(vs)
    #tv_sg = calculate_tv(sg_smoothed)
    
    println("Total Variation of step signal: $tv_step")
    println("Total Variation of PIDSmoothing: $tv_pid")
    println("Total Variation of LOESS: $tv_loess")
    #println("Total Variation of SG: $tv_sg")
end



#=#### test median fiter ##########################
begin
    ##### an example for using PID with limited number of integrals ######################################################################
    using PIDSmoothing
    using Plots
    using Random
    using Statistics
    function med(data::Vector{T}, neighbors::Int) where T<:AbstractFloat
        #dims=1
        len=length(data)
        smoothed=zeros(T, len)
        for i in eachindex(data)
            smoothed[i]=Statistics.median(data[max(1,i-neighbors):min(len, i+neighbors)])
        end
        return smoothed
    end

    Random.seed!(0)
    x = range(0, stop=20, length=200)
    y = sin.(x) .+ randn(length(x)) .* 0.1
    #y=Float32.([(rand(10)*5).+20; rand(100).*2; (rand(50).*5).-50; rand(70).-10; (rand(25).*5).-40; rand(20).*5])
    kp = 0.1
    ki = 0.01
    kd = 0.01
    integral_limit = 2.0
    integral_length = 10
    adaptive_rate=0.01
    n_setpoint=5
    decay=0.2
    filtered = pid_smoothing(y, n_setpoint=n_setpoint,  ki=ki, kp=kp, kd=kd,
                    adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
                    integral_length=integral_length, decay=decay,
                    neighbors_before=true, neighbors_after=true)
    plot(filtered, lw=1, label="PID")
    
    filtered2= pid_smoothing(med(y, n_setpoint), n_setpoint=0,  ki=ki, kp=kp, kd=kd,
    adaptive_rate=adaptive_rate, integral_limit=integral_limit, 
    integral_length=integral_length, decay=decay,
    neighbors_before=true, neighbors_after=true)
    plot!(filtered2, label="PID+Median", lw=2)
    #, 0.1, 0.01, 0.01, 10
    plot!(med(y,5), lw=1, alpha=1.0, label="Median")
    plot!(y, alpha=0.4, label=false)
end
###################################### fitting function test ################################
begin

    function test_fit(data, index, neighbors)

    

        degree = 3
        #index = 1
        

        # Define the range of points to consider for fitting
        start = 1
        finish = length(data)

        # Ensure there are enough points for polynomial fitting
    
        x = Float64.(collect(start:finish))
        y = data

        # Compute weights: closer points get higher weights
        weights = exp.(-abs.(x .- index))  # Exponential decay based on distance from `index`
        #sigma = neighbors / 2  # `sigma` controls the spread
        #weights = exp.(-((x .- index) .^ 2) / (2 * sigma^2))  # `sigma` controls the spread
        #weights = max.(0, 1 .- abs.(x .- index) / neighbors )  # `max_distance` is the cutoff
        # Perform weighted polynomial fitting
        W = zeros(length(weights), length(weights))
        for i in eachindex(weights)# 1:length(weights)
            W[i, i] = weights[i]
        end
        #W = Diagonal(weights)  # Weight matrix
        X = hcat([x .^ i for i in 0:degree-1]...)  # Design matrix for a cubic polynomial
        coeffs = (X' * W * X) \ (X' * W * y)  # Solve weighted least squares

        # Evaluate the fitted polynomial at `index`
        fitted_values = zeros(length(x))
        for i in eachindex(x)
            fitted_values[i] = sum(coeffs .* (i .^ (0:degree-1)))
        end
        return fitted_values
    
    end
    aaa = [55, 65, 12, 89, 65, 35, 48, 21, 8, 11, 20, 9, 0]
    scatter(aaa)
    plot!(test_fit(aaa, 6, 5))

end
=#