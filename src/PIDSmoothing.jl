module PIDSmoothing

    using Statistics

    const DEFAULT_KP = 0.1
    const DEFAULT_KI = 0.01
    const DEFAULT_KD = 0.01
    const DEFAULT_INTEGRAL_LENGTH = 0
    const DEFAULT_INTEGRAL_VALUE_LIMIT=Inf
    const DEFAULT_SETPOINT_NEIGHBORS=0
    const DEFAULT_ADAPTIVE_RATE=0.0
    const DEFAULT_NEIGHBORS_AFTER=true
    const DEFAULT_NEIGHBORS_BEFORE=true
    const DEFAULT_DECAY_COEFFICIENT=0.1
    

    #export get_adaptive_mean_filter
    export pid_smoothing
    export NumberLimitPID
    export ValueLimitPID
    export batch_pid_smoothing
    export weighted_pid_smoothing
    export realtime_pid_smoothing


    ################################### Adaptive PID Smoothing #############################################
    ############ adaptive smoothing with fixed-size integral buffer

    # Define the PID controller struct with adaptive rate and fixed-size integral buffer
    mutable struct NumberLimitPID{T<:AbstractFloat}
        kp::T
        ki::T
        kd::T
        prev_error::T
        integral_buffer::Vector{T}
        integral_length::Int
        adapt_rate::T
        integral_sum::T
        integral_buffer_index::Int
        integral_limit::T
    end

    # Initialize the PID controller with adaptive rate and fixed-size integral buffer
    function NumberLimitPID(kp::T, ki::T, kd::T, integral_length::Int, adapt_rate::T, integral_limit::T)::NumberLimitPID{T} where T<:AbstractFloat
        return NumberLimitPID(kp, ki, kd,  zero(T), zeros(T, integral_length), integral_length, adapt_rate, zero(T), 1, integral_limit)
    end

    
      ############ adaptive smoothing with limited integral value 
    # Define the PID controller struct with adaptive rate and limited integral value 
    mutable struct ValueLimitPID{T<:AbstractFloat}
        kp::T
        ki::T
        kd::T
        prev_error::T
        integral::T
        integral_limit::T
        adapt_rate::T
    end

    # Initialize the PID controller with adaptive rate and limited integral value 
    function ValueLimitPID(kp::T, ki::T, kd::T, integral_limit::T, adapt_rate::T)::ValueLimitPID{T} where T<:AbstractFloat
        return ValueLimitPID(kp, ki, kd, zero(T), zero(T), integral_limit, adapt_rate)
    end

    ### calculating the new integral_sum value
    function update_integral_sum!(pid::NumberLimitPID{T}, error::T; totaldecay::T) where T<:AbstractFloat
        pid.integral_sum+=error
        i=((pid.integral_buffer_index - 1) % pid.integral_length)+1
        if pid.integral_buffer_index > pid.integral_length
            pid.integral_sum -= pid.integral_buffer[i]*totaldecay
        end
        pid.integral_buffer[i]=error
        pid.integral_buffer_index+=1                
    end

    # Update method for the PID controller with adaptive rate and fixed-size integral buffer
    function update!(pid::NumberLimitPID{T}, current_value::T, setpoint::T; decay::T = DEFAULT_DECAY_COEFFICIENT)::T where T<:AbstractFloat
        totaldecay = (1-decay)^pid.integral_length
        error::T = setpoint - current_value
        #pid.integral += error

        # Update the integral buffer
        #=    
        pid.integral_sum+=error
        i=((pid.integral_buffer_index-1)%pid.integral_length)+1
        if pid.integral_buffer_index>pid.integral_length
            pid.integral_sum-=pid.integral_buffer[i]
        end
        pid.integral_buffer[i]=error
        pid.integral_buffer_index+=1
        =#    
        update_integral_sum!(pid, error; totaldecay = totaldecay)
        # Apply anti-windup by clamping the integral term
        # pid.integral_sum=clamp(pid.integral_sum, -pid.integral_limit, pid.integral_limit)
        pid.integral_sum = (1-decay) * pid.integral_sum
        myintegral = clamp(pid.integral_sum, -pid.integral_limit, pid.integral_limit)

        derivative::T = error - pid.prev_error
        output::T = pid.kp * error + pid.ki * myintegral + pid.kd * derivative

        # Adaptive tuning logic
        pid.kp += pid.adapt_rate * error * error
        pid.ki += pid.adapt_rate * error * myintegral
        pid.kd += pid.adapt_rate * error * derivative

        pid.prev_error = error
        return output
    end

    # Update method for the PID controller with adaptive rate and limited integral value 
    function update!(pid::ValueLimitPID{T}, current_value::T, setpoint::T; decay::T = DEFAULT_DECAY_COEFFICIENT)::T where T<:AbstractFloat
        error::T = setpoint - current_value
        pid.integral += error
        
        # Apply anti-windup by clamping the integral term
        pid.integral=clamp(pid.integral, -pid.integral_limit, pid.integral_limit)
        #myintegral = clamp(pid.integral, -pid.integral_limit, pid.integral_limit)
        pid.integral = decay * pid.integral
        
        derivative::T = error - pid.prev_error
        output::T = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative

        # Adaptive tuning logic
        pid.kp += pid.adapt_rate * error * error
        pid.ki += pid.adapt_rate * error * pid.integral
        pid.kd += pid.adapt_rate * error * derivative

        pid.prev_error = error
        return output
    end

    #=
    function adaptive_mean_filter!(smoothed::TA, data::TA, win_widt=5) where {TA <: AbstractArray} 
            mysum = zero(eltype(smoothed))
            # elements in the buffer
            count = 0
            src_n = 1
            for n in eachindex(smoothed)
                if (count < length(data) - win_width)
                    count += 1
                    mysum += data[src_n]
                    src_n += 1
                    if (count < win_width)
                        count += 1
                        mysum += data[src_n]
                        src_n += 1
                    end
                end
                if (count > win_width)
                    mysum -= data[n-win_width]
                    src_n += 1
                    count -= 1
                    if (count > length(data) - win_width)
                        count -= 1
                        mysum -= data[src_n]
                        src_n += 1
                    end
                end                
                smoothed[n] = mysum / count
            end
            return smoothed
    end    
    function get_adaptive_mean_filter(data::TA, win_width=5) where {TA <: AbstractArray} 
        mysum = zero(eltype(data))
        # elements in the buffer
        count = 0
        r_src_n = 1
        l_src_n = 1
        n = 1
        if iseven(win_width)
            error("only uneven windows allowed")
        end
        win_ctr = (win_width-1)÷2+1
        function update!()
            if (r_src_n <= length(data))
                count += 1
                mysum += data[r_src_n]
                r_src_n += 1
                if (n > 1 && n <= win_ctr)
                    count += 1
                    mysum += data[r_src_n]
                    r_src_n += 1
                end
            end
            if (n > win_ctr)
                mysum -= data[l_src_n]
                l_src_n += 1
                count -= 1
                if (n > 1 && n > length(data) - win_ctr + 1)
                    count -= 1
                    mysum -= data[l_src_n]
                    l_src_n += 1
                end
            end                
            n += 1
            #@show mysum 
            #@show count
            #@show n
            @show r_src_n
            @show l_src_n
            return mysum / count
        end
        return update!
    end
    =#
    ### Calculating the new setpoint value
    function get_adaptive_mean_filter!(data::TA, neighbors::Int=DEFAULT_SETPOINT_NEIGHBORS; neighbors_before::Bool=true, neighbors_after::Bool=true) where TA <: AbstractArray

        mysum = zero(eltype(data))
        # elements in the buffer
        count = 0 ## number of added-up elements so far
        r_src_n = 1 ## the right side of the window
        l_src_n = 1 ## the left side of the window
        n = 1 ## the current index of the data array
        win_ctr = neighbors+1

        function update!()
            if !neighbors_before && !neighbors_after
                n+=1
                return data[n-1]
            end
            if (r_src_n <= length(data))
                
                count += 1
                mysum += data[r_src_n]
                r_src_n += 1
                
                if (n > 1 && n <= win_ctr && neighbors_after && neighbors_before)
                    count += 1
                    mysum += data[r_src_n]
                    r_src_n += 1
                end
            end
            if (n > win_ctr)
                      
                mysum -= data[l_src_n]
                l_src_n += 1
                count -= 1
                
                if (n > 1 && n > length(data) - win_ctr + 1 && neighbors_after && neighbors_before)
                    count -= 1
                    mysum -= data[l_src_n]
                    l_src_n += 1
                end
            end                
            n += 1
            return mysum / count
        end
        if !neighbors_before && neighbors_after
            for i in 1:neighbors
                update!()
            end
        end
        return update!
    end


    # PID smoothing function with adaptive rate and fixed-size integral buffer
    """
    Apply PID smoothing 

    # Arguments
    - `data::Vector{AbstarctFloat}`: The input data to be smoothed.
    - `kp::AbstarctFloat` is the gain on the error (default 0.1).
    - `ki::AbstarctFloat` is the gain on the integral (default 0.01).
    - `kd::AbstarctFloat` is the gain on the derivative (default 0.01).
    - `integral_length::Int`: The limit for the number of saved integrals (default Inf).
    - `integral_limit::AbstractFloat` is the maximum absolute value to clamp the integral (default=Inf).
    - `decay::AbstractFloat`: is the decaying facor of the integral value (default=0.1)
    - `n_setpoint::Int` is the number of neighboring datapoints before or after the coming datapoint to calculate the new setpoint (default=0).
    - `adaptive_rate::AbstractFloat` is the adaptive rate to change the kp, ki and kd (default=0).
    - `neighbors_before::Bool` is used to turn on or off the effect of neighbors before the coming datapoint in the new setpoint calculation (default=true).
    - `neighbors_after::Bool` is used to turn on or off the effect of neighbors after the coming datapoint in the new setpoint calculation (default=true).
   


    # Returns
    - `Vector{AbstarctFloat}`: The smoothed data.

    # Example
    ```julia
    julia> using PIDSmoothing

    julia> data=[1.0, 2.5, 1.4, 3.5, 3.0, 4.2, 5.0, 6.1, 6.8, 8.1]
    10-element Vector{Float64}:
    1.0
    2.5
    1.4
    ⋮
    6.8
    8.1

    julia> smoothed_data = pid_smoothing(data, kp=0.1, ki=0.01, kd=0.01, n_setpoint=2, integral_limit=2.0, integral_length=3)
    10-element Vector{Float64}:
    1.0
    1.076
    1.22048
    ⋮
    3.132026005399606
    3.6552416432751693
    ```
    """
    function pid_smoothing(data::Vector{T}; kp::AbstractFloat=DEFAULT_KP, ki::AbstractFloat=DEFAULT_KI, kd::AbstractFloat=DEFAULT_KD,  ###adaptive_nlimit_smoothing
                                    integral_length::Int=DEFAULT_INTEGRAL_LENGTH, n_setpoint::Int=DEFAULT_SETPOINT_NEIGHBORS,
                                    adaptive_rate::AbstractFloat=DEFAULT_ADAPTIVE_RATE, neighbors_before::Bool=DEFAULT_NEIGHBORS_BEFORE,
                                    neighbors_after::Bool=DEFAULT_NEIGHBORS_AFTER, decay::AbstractFloat=DEFAULT_DECAY_COEFFICIENT,
                                    integral_limit::AbstractFloat=DEFAULT_INTEGRAL_VALUE_LIMIT)::Vector{T} where T<:AbstractFloat

        kp = convert(T, kp)
        ki = convert(T, ki)
        kd = convert(T, kd)
        adaptive_rate=convert(T, adaptive_rate)
        integral_limit = convert(T, integral_limit)
        decay=convert(T, decay)


        smoothed_data = zeros(T, length(data))

        if integral_length==0
            pid = ValueLimitPID(kp, ki, kd, integral_limit, adaptive_rate)
        else
            integral_length=min(integral_length, length(data))
            pid = NumberLimitPID(kp, ki, kd, integral_length, adaptive_rate, integral_limit)
        end
      

        smoothed_data[1] = data[1]
        
        #up=get_adaptive_mean_filter(data, 2*n_setpoint+1)
        up=get_adaptive_mean_filter!(data, n_setpoint, neighbors_before=neighbors_before, neighbors_after=neighbors_after )
        up()
        all_set = []
        for i in 2:length(data)
            s = up()
            push!(all_set, s)
            smoothed_data[i] = smoothed_data[i-1] + update!(pid, smoothed_data[i-1], s, decay=decay)
        end
        
       
        return smoothed_data#, all_set
    end
  
    ################################ batch smoothing with fixed-size integral buffer 
    """
    Apply PID smoothing on a batch of data

    # Arguments
    - `data_series::Vector{Vector{AbstarctFloat}}`: The input data to be smoothed.
    - `kp::AbstarctFloat` is the gain on the error (default 0.1).
    - `ki::AbstarctFloat` is the gain on the integral (default 0.01).
    - `kd::AbstarctFloat` is the gain on the derivative (default 0.01).
    - `integral_length::Int`: The limit for the number of saved integrals (default Inf).
    - `integral_limit::AbstractFloat` is the maximum absolute value to clamp the integral (default=Inf).
    - `decay::AbstractFloat`: is the decaying facor of the integral value (default=0.1)
    - `n_setpoint::Int` is the number of neighboring datapoints before or after the coming datapoint to calculate the new setpoint (default=0).
    - `adaptive_rate::AbstractFloat` is the adaptive rate to change the kp, ki and kd (default=0).
    - `neighbors_before::Bool` is used to turn on or off the effect of neighbors before the coming datapoint in the new setpoint calculation (default=true).
    - `neighbors_after::Bool` is used to turn on or off the effect of neighbors after the coming datapoint in the new setpoint calculation (default=true).
   


    # Returns
    - `::Vector{Vector{AbstarctFloat}}`: The smoothed data batch.

    # Example
    ```julia
    julia> using PIDSmoothing

    julia> data_batch=[[1.0, 2.0, 3.0 , 4.0],[10.0, 20.0, 15.0, 10.0]]
    2-element Vector{Vector{Float64}}:
    [1.0, 2.0, 3.0, 4.0]
    [10.0, 20.0, 15.0, 10.0]

    julia> batch_pid_smoothing(data_batch)
    2-element Vector{Vector{Float64}}:
    [1.0, 1.12, 1.3456000000000001, 1.674128]
    [10.0, 11.2, 11.655999999999999, 11.557279999999999]
    ```
    """
    function batch_pid_smoothing(data_series::Vector{Vector{T}}; kp::AbstractFloat=DEFAULT_KP, 
                                    ki::AbstractFloat=DEFAULT_KI, kd::AbstractFloat=DEFAULT_KD, adaptive_rate::AbstractFloat=DEFAULT_ADAPTIVE_RATE, 
                                    integral_length::Int=DEFAULT_INTEGRAL_LENGTH, n_setpoint::Int=DEFAULT_SETPOINT_NEIGHBORS, 
                                    integral_limit::AbstractFloat=DEFAULT_INTEGRAL_VALUE_LIMIT, decay::AbstractFloat=DEFAULT_DECAY_COEFFICIENT,
                                    neighbors_before::Bool=DEFAULT_NEIGHBORS_BEFORE, neighbors_after::Bool=DEFAULT_NEIGHBORS_AFTER)::Vector{Vector{T}} where T<:AbstractFloat
        kp = convert(T, kp)
        ki = convert(T, ki)
        kd = convert(T, kd)
        adaptive_rate=convert(T, adaptive_rate)
        integral_limit = convert(T, integral_limit)

        return [pid_smoothing(data; kp=kp, ki=ki, kd=kd, integral_length=integral_length, integral_limit=integral_limit,
                                                n_setpoint=n_setpoint, adaptive_rate=adaptive_rate, decay=decay,
                                                neighbors_after=neighbors_after, neighbors_before=neighbors_before) for data in data_series]
    end

    ############################## Custom weight integral error smoothing 
    
    # Update method for the PID controller with fixed-size integral buffer along with a weight vector
    function weighted_integral_update!(pid::NumberLimitPID{T}, current_value::T, setpoint::T, weights::Vector{T}, decay::T)::T where T<:AbstractFloat
        
        error::T = setpoint - current_value
        push!(pid.integral_buffer, error)
        if length(pid.integral_buffer) > pid.integral_length
            popfirst!(pid.integral_buffer)
        end
        #pid.integral_buffer*=(1-decay)
        integral_sum::T = sum(pid.integral_buffer .* weights.*(1-decay))

        # Apply anti-windup by clamping the integral term
        integral_sum=clamp(integral_sum, -pid.integral_limit, pid.integral_limit)

        derivative::T = error - pid.prev_error
        output::T = pid.kp * error + pid.ki * integral_sum + pid.kd * derivative

        # Adaptive tuning logic
        pid.kp += pid.adapt_rate * error * error
        pid.ki += pid.adapt_rate * error * integral_sum
        pid.kd += pid.adapt_rate * error * derivative

        pid.prev_error = error
        return output
    end

    # PID smoothing function with fixed-size integral buffer along with a weight vector
    """
    Apply PID smoothing with a weight vector for the integrals

    # Arguments
    - `data::Vector{AbstarctFloat}`: The input data to be smoothed.
    - `kp::AbstarctFloat` is the gain on the error (default 0.1).
    - `ki::AbstarctFloat` is the gain on the integral (default 0.01).
    - `kd::AbstarctFloat` is the gain on the derivative (default 0.01).
    - `integral_length::Int`: The limit for the number of saved integrals (default Inf).
    - `integral_limit::AbstractFloat` is the maximum absolute value to clamp the integral (default=Inf).
    - `decay::AbstractFloat`: is the decaying facor of the integral value (default=0.1)
    - `weights::Vector{AbstarctFloat}` is the weight vector for integral. its length should be equal to the integral_length.
    - `n_setpoint::Int` is the number of neighboring datapoints before or after the coming datapoint to calculate the new setpoint (default=0).
    - `adaptive_rate::AbstractFloat` is the adaptive rate to change the kp, ki and kd (default=0).
    - `neighbors_before::Bool` is used to turn on or off the effect of neighbors before the coming datapoint in the new setpoint calculation (default=true).
    - `neighbors_after::Bool` is used to turn on or off the effect of neighbors after the coming datapoint in the new setpoint calculation (default=true).
   


    # Returns
    - `Vector{AbstarctFloat}`: The smoothed data.

    # Example
    ```julia
    julia> using PIDSmoothing

    julia> data=[1.0, 2.5, 1.4, 3.5, 3.0, 4.2, 5.0, 6.1, 6.8, 8.1]
    10-element Vector{Float64}:
    1.0
    2.5
    1.4
    3.5
    3.0
    4.2
    5.0
    6.1
    6.8
    8.1

    julia> weights=Float64.(collect(0:(1/3):1))
    4-element Vector{Float64}:
    0.0
    0.3333333333333333
    0.6666666666666666
    1.0

    julia> weighted_pid_smoothing(data, weights=weights, integral_length=4)
    10-element Vector{Float64}:
    1.0
    1.1800000000000002
    1.2014
    1.4814986666666667
    1.6567901600000001
    1.964575669688889
    2.325410894304
    2.776720872019816
    3.257050484792822
    3.8373754598767356
    ```
    """
    function weighted_pid_smoothing(data::Vector{T};  weights::Vector{T}, kp::AbstractFloat=DEFAULT_KP, ### pid_nlimit_weighted
                                  ki::AbstractFloat=DEFAULT_KI, kd::AbstractFloat=DEFAULT_KD, n_setpoint::Int=DEFAULT_SETPOINT_NEIGHBORS, 
                                  adaptive_rate::AbstractFloat=DEFAULT_ADAPTIVE_RATE, neighbors_before::Bool=DEFAULT_NEIGHBORS_BEFORE,
                                  neighbors_after::Bool=DEFAULT_NEIGHBORS_AFTER,  decay::AbstractFloat=DEFAULT_DECAY_COEFFICIENT,
                                  integral_limit::AbstractFloat=DEFAULT_INTEGRAL_VALUE_LIMIT)::Vector{T} where T<:AbstractFloat

        kp = convert(T, kp)
        ki = convert(T, ki)
        kd = convert(T, kd)
        adaptive_rate=convert(T, adaptive_rate)
        integral_limit = convert(T, integral_limit)
        decay=convert(T, decay)
        #weights=convert.(T, weights)                  
        
        smoothed_data = zeros(T, length(data))

        integral_length=length(weights)
        if integral_length>length(data)
            error("The length of the weights vector should be smaller than the length of whole data.")
        end
        
        pid = NumberLimitPID(kp, ki, kd, integral_length, adaptive_rate, integral_limit)
        smoothed_data[1] = data[1]  # Initialize the first data point

        #up=get_adaptive_mean_filter(data, 2*n_setpoint+1)
        up=get_adaptive_mean_filter!(data, n_setpoint, neighbors_before=neighbors_before, 
                                     neighbors_after=neighbors_after)
        up()
        for i in 2:length(data)
            smoothed_data[i] = smoothed_data[i-1] + weighted_integral_update!(pid, smoothed_data[i-1], up(), weights, decay)
        end
             
        #=
        for i in 2:length(data)
            start=max(1,i-(n_setpoint#=-1=#)*neighbors_before)
            finish=min(size(data)[1], i+(n_setpoint#=-1=#)*neighbors_after)
            if abs(start-i)>abs(finish-i) && finish==size(data)[1]
                start=max(1,i-abs(finish-i))
            end
            set_point::T=mean(data[start:finish])
            smoothed_data[i] = smoothed_data[i-1] + weighted_integral_update!(pid, smoothed_data[i-1], set_point, weights) 
        end
        =#
        return smoothed_data
    end

    ###################################### real time PID smoothing  #############################
    """
    Apply realtime PID smoothing to the new arrivein datapoint

    # Arguments
    - `pid::Union{NumberLimitPID, ValueLimitPID}`: The PID struct which works as smoother
    - `new_data_points::Vector{AbstarctFloat}`: The new_data_point is a vector which its average will be used as the new setpoint.
    - `previous_smoothed_value::AbstractFloat`: Previous smoothed value.
    - `decay::AbstractFloat`: is the decaying facor of the integral value (default=0.1)

    # Returns
    - `AbstractFloat`: The smoothed datapoint.

    # Example
    ```julia
    julia> using PIDSmoothing

    julia> kp = 0.2; ki = 0.001; kd = 0.1; integral_limit =1.0; integral_length=10; adaptive_rate=0.0002;

    julia> pid = NumberLimitPID(kp, ki, kd, integral_length, adaptive_rate, integral_limit)
    NumberLimitPID{Float64}(0.2, 0.001, 0.1, 0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10, 0.0002, 0.0, 1, 1.0)

    julia> new_arriving_datapoint=20.0
    20.0

    julia> new_data_points=[17.0, 19.0, new_arriving_datapoint]
    3-element Vector{Float64}:
    17.0
    19.0
    20.0

    julia> realtime_pid_smoothing(pid, new_data_points, previous_smoothed_value)
    18.550166666666666
    ```
    """
    function realtime_pid_smoothing(pid::Union{NumberLimitPID{T}, ValueLimitPID{T}},
                                    new_data_points::Vector{T}, previous_smoothed_value::T;
                                    decay::AbstractFloat=DEFAULT_DECAY_COEFFICIENT)::T where T<:AbstractFloat

        
        #pid.kp = convert(T, pid.kp)
        #pid.ki = convert(T, pid.ki)
        #pid.kd = convert(T, pid.kd)
        #if typeof(pid)==:NumberLimitPID
         #   pid.adaptive_rate=convert(T, pid.adaptive_rate)
        #end
        #pid.integral_limit = convert(T, pid.integral_limit)
        decay=convert(T, decay)
        setpoint::T=mean(new_data_points)
        return previous_smoothed_value + update!(pid, previous_smoothed_value, setpoint, decay=decay)
    end      

end # module PIDSmoothing 



################################################# deleted codes ################################

    #=
    #********************************************************************************
    ##################################### Basic PID smoothing ########################################
    ################## Smoothing with fixed-size integral buffer to resolve the integral windup problem
    # Define the PID controller struct with fixed-size integral buffer
    mutable struct PID
        kp::Float64
        ki::Float64
        kd::Float64
        prev_error::Float64
        integral_buffer::Vector{Float64}
        integral_length::Int
        integral_sum::Float64
        integral_buffer_index::Int
    end

    # Initialize the PID controller with fixed-size integral buffer
    function PID(kp::Float64, ki::Float64, kd::Float64, integral_length::Int)
        return PID(kp, ki, kd, 0.0, zeros(Float64, integral_length), integral_length, 0.0, 1)
    end

    # Update method for the PID controller with fixed-size integral buffer
    function update!(pid::PID, current_value::Float64, setpoint::Float64)
        error = setpoint - current_value
        
        # Update the integral buffer
       
        pid.integral_sum+=error
        i=((pid.integral_buffer_index-1)%pid.integral_length)+1
        if pid.integral_buffer_index>pid.integral_length
            pid.integral_sum-=pid.integral_buffer[i]
        end
        pid.integral_buffer[i]=error
        pid.integral_buffer_index+=1

        derivative = error - pid.prev_error
        output = pid.kp * error + pid.ki * pid.integral_sum + pid.kd * derivative
        pid.prev_error = error
        return output
    end

    # PID smoothing function with fixed-size integral buffer
    """
    Apply PID smoothing with with fixed-size integral buffer

    # Arguments
    - `data::Vector{Float64}`: The input data to be smoothed.
    - `integral_length::Float64`: The limit for the number of saved integrals.

    # Returns
    - `Vector{Float64}`: The smoothed data.

    # Example
    ```julia
    using PIDSmoothing;

    - data = [1.0, 2.0, 3.0, 4.0, 5.0];
    - limit = 10;
    - smoothed_data = pid_smoothing_nlimit(data, integral_length=limit);
    - println(smoothed_data);
    """
    function pid_smoothing_nlimit(data::Vector{Float64}; kp::Float64=DEFAULT_KP, ki::Float64=DEFAULT_KI,
                                  kd::Float64=DEFAULT_KD, integral_length::Int=DEFAULT_INTEGRAL_LENGTH, n_setpoint::Int=DEFAULT_SETPOINT_NUMBERS)
        smoothed_data = zeros(length(data))
        integral_length=min(integral_length, length(data))
        pid = PID(kp, ki, kd, integral_length)
        smoothed_data[1] = data[1]  # Initialize the first data point
        
        for i in 2:length(data)
            smoothed_data[i] = smoothed_data[i-1] + update!(pid, smoothed_data[i-1], mean(data[max(i-n_setpoint+1,1):i]))
        end
        
        return smoothed_data
    end

    
    ################## Smoothing with limited integral value to resolve the integral windup problem
    # Define the PID controller struct as mutable with limited integral value
    mutable struct PID1{T<:AbstractFloat}
        kp::T
        ki::T
        kd::T
        prev_error::T
        integral::T
        integral_limit::T
    end

    # Initialize the PID controller with limited integral value
    function PID1(kp::T, ki::T, kd::T, integral_limit::T) where T<:AbstractFloat
        return PID1{T}(kp, ki, kd, zero(T), zero(T), integral_limit)
    end

    # Update method for the PID controller with limited integral value
    function update!(pid::PID1{T}, current_value::T, setpoint::T) where T<:AbstractFloat
        error::T = setpoint - current_value
        pid.integral += error

        # Apply anti-windup by clamping the integral term
        pid.integral=clamp(pid.integral, -pid.integral_limit, pid.integral_limit)

        derivative::T = error - pid.prev_error
        output::T = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative
        pid.prev_error = error
        return output
    end

    # PID smoothing function with limited integral value
    function pid_smoothing_vlimit(data::Vector{T}; kp::AbstractFloat=DEFAULT_KP, ki::AbstractFloat=DEFAULT_KI, kd::AbstractFloat=DEFAULT_KD,
                                 integral_limit::AbstractFloat=DEFAULT_INTEGRAL_VALUE_LIMIT, n_setpoint::Int=DEFAULT_SETPOINT_NUMBERS)::Vector{T} where T<:AbstractFloat

        # Convert parameters to type T
        kp = convert(T, kp)
        ki = convert(T, ki)
        kd = convert(T, kd)
        integral_limit = convert(T, integral_limit)
        
        smoothed_data = zeros(T,length(data))
        pid = PID1(kp, ki, kd, integral_limit)
        #pid.integral_limit=integral_limit
        smoothed_data[1] = data[1]

        for i in 2:length(data)
            set_point::T=mean(data[max(1,i-n_setpoint+1):i])
            smoothed_data[i] = smoothed_data[i-1] + update!(pid, smoothed_data[i-1], set_point) # data[i])  # 
        end
        
        return smoothed_data
    end
    #*****************************************************************************************
    =#

    
    #=
    #*********************************************************************************************************
    # PID smoothing function with adaptive rate and limited integral value 
    function adaptive_vlimit_smoothing(data::Vector{T}; kp::AbstractFloat=DEFAULT_KP, ki::AbstractFloat=DEFAULT_KI, kd::AbstractFloat=DEFAULT_KD, 
                                    integral_limit::AbstractFloat=DEFAULT_INTEGRAL_VALUE_LIMIT, n_setpoint::Int=DEFAULT_SETPOINT_NEIGHBORS,
                                    adaptive_rate::AbstractFloat=DEFAULT_ADAPTIVE_RATE, neighbors_before::Bool=DEFAULT_NEIGHBORS_BEFORE,
                                    neighbors_after::Bool=DEFAULT_NEIGHBORS_AFTER)::Vector{T} where T<:AbstractFloat

        kp = convert(T, kp)
        ki = convert(T, ki)
        kd = convert(T, kd)
        adaptive_rate=convert(T, adaptive_rate)
        integral_limit = convert(T, integral_limit)
        
        smoothed_data = zeros(T, length(data))
        pid = AdaptivePID1(kp, ki, kd, integral_limit, adaptive_rate)
        #pid.integral_limit=integral_limit
        #pid.adaptive_rate=adaptive_rate

        smoothed_data[1] = data[1]

        for i in 2:length(data)
            start=max(1,i-(n_setpoint#=+1=#)*neighbors_before)
            finish=min(size(data)[1], i+(n_setpoint#=-1=#)*neighbors_after)
            if abs(start-i)>abs(finish-i) && finish==size(data)[1]
                start=max(1,i-abs(finish-i))
            end
            set_point::T=mean(data[start:finish])
            smoothed_data[i] = smoothed_data[i-1] + update!(pid, smoothed_data[i-1],set_point) # data[i])  # 
        end

        #=
        for i in 2:length(data)
            smoothed_data[i] = smoothed_data[i-1] + update!(pid, smoothed_data[i-1], mean(data[max(1,i-n_setpoint+1):i])) # data[i])  # 
        end
        =#
        return smoothed_data
    end
    =#

    #=
    #*************************************************************************************************
    ################################################# batch PID smoothing #########################################
    ################################ batch smoothing with limited integral value
    function batch_vlimit_smoothing(data_series::Vector{Vector{T}}; kp::AbstractFloat=DEFAULT_KP,
                                    ki::AbstractFloat=DEFAULT_KI, kd::AbstractFloat=DEFAULT_KD, 
                                    integral_limit::AbstractFloat=DEFAULT_INTEGRAL_VALUE_LIMIT, n_setpoint::Int=DEFAULT_SETPOINT_NEIGHBORS, 
                                    adaptive_rate::AbstractFloat=DEFAULT_ADAPTIVE_RATE,
                                    neighbors_before::Bool=DEFAULT_NEIGHBORS_BEFORE, neighbors_after::Bool=DEFAULT_NEIGHBORS_AFTER)::Vector{Vector{T}} where T<:AbstractFloat
        kp = convert(T, kp)
        ki = convert(T, ki)
        kd = convert(T, kd)
        adaptive_rate=convert(T, adaptive_rate)
        integral_limit = convert(T, integral_limit)    

        return [adaptive_vlimit_smoothing(data; kp=kp, ki=ki, kd=kd, integral_limit=integral_limit, 
                                          n_setpoint=n_setpoint, adaptive_rate=adaptive_rate,
                                          neighbors_after=neighbors_after, neighbors_before=neighbors_before) for data in data_series]
    end
    #*****************************************************************************************************************
    =#