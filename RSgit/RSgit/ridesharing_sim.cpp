#include "ridesharing_sim.h"

//constructor
//initialize all necessary variables
ridesharing_sim::ridesharing_sim(ULL param_N, ULL param_B, ULL seed) : network(param_N, random_generator), measurements(network), out(std::ofstream("default_output.dat"))
{
	//see random generator
	random_generator.seed(seed);

	//setup list of transporters
	transporter_list = std::vector<transporter>(param_B, transporter(-1, 0, 0, random_generator));

	//reset all important variables
	time = 0;
	total_requests = 0;
	total_serviced_requests = 0;

	while (!transporter_event_queue.empty())
		transporter_event_queue.pop();

	normalized_request_rate = 1;
	request_rate = 1;

	next_request_time = 0;

	//default: disable and reset measurements
	disable_measurements();
	disable_timeseries_output();

	start_of_measured_time = 0;
	start_of_measured_total_requests = 0;
	start_of_measured_serviced_requests = 0;

	start_of_output_time = 0;

	measurements.reset();
}

ridesharing_sim::~ridesharing_sim()
{
	//clear transporter list and close output file
	transporter_list.clear();
	if (out.is_open())
		out.close();
}

//reset all measurements and disable new measurements
void ridesharing_sim::reset_measurements()
{
	measurements.reset();
	disable_measurements();
}

//output measurements
void ridesharing_sim::print_measurements(std::ofstream& out)
{
	measurements.print(out, true);
}

//output parameters of the simulation
void ridesharing_sim::print_params(std::ofstream& out, bool readable)
{
	double total_velocity = 0;
	for (transporter& t : transporter_list)
		total_velocity += t.get_velocity();

	if (!readable)
	{
		out << time - start_of_measured_time << '\t' << total_requests - start_of_measured_total_requests << '\t' << total_serviced_requests - start_of_measured_serviced_requests << '\t' 		//simulation time
			<< transporter_list.size() << '\t' << request_rate << '\t' << normalized_request_rate << '\t'		//simulation parameters
			<< network.get_mean_pickup_distance() << '\t' << network.get_mean_dropoff_distance() << '\t' << network.get_request_asymmetry() << '\t'	//network parameters
			<< total_velocity << '\t';
	}
	else
	{
		out << "PARAMS" << std::endl << std::endl <<
			"time - start_of_measured_time" << std::endl << time - start_of_measured_time << std::endl <<
			"total_requests - start_of_measured_total_requests" << std::endl << total_requests - start_of_measured_total_requests << std::endl <<
			"total_serviced_requests - start_of_measured_serviced_requests" << std::endl << total_serviced_requests - start_of_measured_serviced_requests << std::endl <<
			"transporter_list.size()" << std::endl << transporter_list.size() << std::endl <<
			"request_rate" << std::endl << request_rate << std::endl <<
			"normalized_request_rate" << std::endl << normalized_request_rate << std::endl <<
			"network.get_mean_pickup_distance()" << std::endl << network.get_mean_pickup_distance() << std::endl <<
			"network.get_mean_dropoff_distance()" << std::endl << network.get_mean_dropoff_distance() << std::endl <<
			"network.get_request_asymmetry()" << std::endl << network.get_request_asymmetry() << std::endl <<
			"total_velocity" << std::endl << total_velocity << std::endl <<
			std::endl;
	}
}

//set normalized request rate and corresponding absolute request rate
void ridesharing_sim::set_normalized_request_rate(double param_normalized_request_rate)
{
	assert(transporter_list.size() > 0);
	assert(param_normalized_request_rate > 0);

	normalized_request_rate = param_normalized_request_rate;

	double total_velocity = 0;
	for (transporter& t : transporter_list)
		total_velocity += t.get_velocity();

	//this is the old definition using pickup + dropoff distance (in the symmetric case, this results in a factor two to the normalized request rate)
	//	request_rate = total_velocity * normalized_request_rate / ( network.get_mean_pickup_distance() + network.get_mean_dropoff_distance() );

	//this definition agrees with the paper (arXiv:1908.05929)
	//normalized request rate > 1 requires ride sharing
	request_rate = total_velocity * normalized_request_rate / network.get_mean_dropoff_distance();
}

//set absolute request rate and corresponding normalized request rate
void ridesharing_sim::set_request_rate(double param_request_rate)
{
	assert(transporter_list.size() > 0);
	assert(param_request_rate > 0);

	request_rate = param_request_rate;

	double total_velocity = 0;
	for (transporter& t : transporter_list)
		total_velocity += t.get_velocity();

	//this is the old definition using pickup + dropoff distance (in the symmetric case, this results in a factor two to the normalized request rate)
	//	normalized_request_rate = request_rate * ( network.get_mean_pickup_distance() + network.get_mean_dropoff_distance() ) / total_velocity;

	//this definition agrees with the paper (arXiv:1908.05929)
	//normalized request rate > 1 requires ride sharing
	normalized_request_rate = request_rate * network.get_mean_dropoff_distance() / total_velocity;
}

//reset number of buses and all other parameters like starting a new simulation
void ridesharing_sim::reset_number_of_buses(ULL param_number_of_buses)
{
	transporter_list.clear();
	transporter_list = std::vector<transporter>(param_number_of_buses, transporter(-1, 0, 0, random_generator));

	time = 0;
	total_requests = 0;
	total_serviced_requests = 0;

	while (!transporter_event_queue.empty())
		transporter_event_queue.pop();

	next_request_time = 0;

	reset_measurements();
	disable_measurements();
	disable_timeseries_output();

	start_of_measured_time = 0;
	start_of_measured_total_requests = 0;
	start_of_measured_serviced_requests = 0;

	start_of_output_time = 0;

	measurements.reset(); // <-- doppelt??
}

//turn on measurements with a given time step
void ridesharing_sim::enable_measurements(double param_measurement_time_step)
{
	measurements.reset();

	assert(param_measurement_time_step > 0);
	if (param_measurement_time_step <= 0)
	{
		do_measurement = false;
	}
	else {
		do_measurement = true;
		measurement_time_step = param_measurement_time_step;
		next_measurement_time = time + measurement_time_step;

		start_of_measured_time = time;
		start_of_measured_total_requests = total_requests;
		start_of_measured_serviced_requests = total_serviced_requests;
	}
}

//turn off measurements
void ridesharing_sim::disable_measurements()
{
	do_measurement = false;
}

//turn on timeseries output
void ridesharing_sim::enable_timeseries_output(double param_output_time_step, std::string output_filename)
{
	assert(param_output_time_step > 0);
	if (param_output_time_step <= 0)
	{
		do_timeseries_output = false;
	}
	else {
		do_timeseries_output = true;
		output_time_step = param_output_time_step;
		next_output_time = time + output_time_step;

		out.close();
		out.open(output_filename.c_str());

		start_of_output_time = time;
	}
}

//output current state for time series output
void ridesharing_sim::output(double output_time)
{
	ULL number_of_idle_transporters = 0;
	ULL total_occupancy = 0;
	ULL total_planned_stops = 0;
	for (transporter t : transporter_list)
	{
		total_occupancy += t.get_occupancy();

		if (t.is_idle())
		{
			++number_of_idle_transporters;
			total_planned_stops += 0;
		}
		else
		{
			number_of_idle_transporters += 0;
			total_planned_stops += t.get_number_of_planned_stops();
		}
	}

	out << output_time - start_of_output_time << '\t'
		<< output_time << '\t'
		<< total_occupancy / (double)transporter_list.size() << '\t'
		<< total_planned_stops / (double)transporter_list.size()
		<< '\t' << number_of_idle_transporters / (double)transporter_list.size()
		<< std::endl;
}

//turn off timeseries output
void ridesharing_sim::disable_timeseries_output()
{
	do_timeseries_output = false;
}

//simulate for a fixed number of requests
void ridesharing_sim::run_sim_requests(ULL sim_requests)
{
	ULL max_requests = total_requests + sim_requests;

	while (total_requests < max_requests)
	{
		time = execute_next_event();
	}

}

//simulate for a fixed time
void ridesharing_sim::run_sim_time(long double sim_time)
{
	double max_time = time + sim_time;

	while (time < max_time)
	{
		time = execute_next_event();
	}

}

//main simulation step (executes the next event and updates the scheduled events accordingly)
double ridesharing_sim::execute_next_event()
{
	double event_time;

	// geaend.


	//if the next event is an output event (and output is enabled)
	if (do_timeseries_output &&
		next_output_time < next_request_time &&
		(transporter_event_queue.empty() || next_output_time < transporter_event_queue.top().first) &&
		(!do_measurement || next_output_time < next_measurement_time)
		)
	{
		event_time = next_output_time;
		output(event_time);

		next_output_time += output_time_step;
	}
	//if the next event is a measurement event (and measurement is enabled)
	else if (do_measurement &&
		next_measurement_time < next_request_time &&
		(transporter_event_queue.empty() || next_measurement_time < transporter_event_queue.top().first)
		)
	{
		event_time = next_measurement_time;
		measurements.measure_system_status(transporter_list, event_time);

		next_measurement_time += measurement_time_step;
	}
	//if the next event is a new_request event
	else if (transporter_event_queue.empty() || next_request_time < transporter_event_queue.top().first)
	{
		offer current_offer;
		offer current_best_offer;

		ULL request_origin;
		ULL request_destination;

		ULL event_transporter_index;
		double next_transporter_event;

		event_time = next_request_time;
		++total_requests;
		std::tie(request_origin, request_destination) = network.generate_request();

		current_best_offer = offer();
		//find the best offer for the request
		for (transporter& t : transporter_list)
		{
			current_offer = t.best_offer(request_origin, request_destination, event_time, network, current_best_offer);
			if (current_offer.is_better_offer)
			{
				assert(current_offer.dropoff_time <= current_best_offer.dropoff_time);
				current_best_offer = current_offer;
			}
		}

		//assign the request to the best transporter and update the events
		event_transporter_index = current_best_offer.transporter_index;
		next_transporter_event = transporter_list[event_transporter_index].assign_customer(
			event_time,
			customer(request_origin, request_destination, event_time, network, transporter_list[event_transporter_index], current_best_offer),
			current_best_offer,
			network
		);
		if (next_transporter_event >= event_time)
			transporter_event_queue.push(std::make_pair(next_transporter_event, event_transporter_index));

		//update event queue with the next request (exponential distribution with mean 1/request rate)
		next_request_time = event_time + exp_dist(random_generator) / request_rate;

	}
	else {	//the next event is a bus event (bus arriving at a node along its route)
		ULL event_transporter_index;
		double next_transporter_event;

		event_time = transporter_event_queue.top().first;
		event_transporter_index = transporter_event_queue.top().second;
		transporter_event_queue.pop();

		//execute event and handle the next event of the transporter (if any)
		next_transporter_event = transporter_list[event_transporter_index].execute_event(event_time, network, measurements, total_serviced_requests, do_measurement);

		//update event queue with the next event of the transporter
		if (next_transporter_event >= event_time)
			transporter_event_queue.push(std::make_pair(next_transporter_event, event_transporter_index));
	}
	return(event_time);
}

//simulate for all requests in the predetermined list
//same as above but not using random events
void ridesharing_sim::run_sim_request_list(std::list< std::pair< double, std::pair<ULL, ULL> > > request_list)
{
	double event_time;

	//find time of last request
	double max_time = 0;
	next_request_time = 0;
	if (!request_list.empty())
	{
		max_time = request_list.rbegin()->first;
		next_request_time = request_list.begin()->first;
	}

	//simulate until last request (does not finish serving all requests!)
	while (time < max_time)
	{
		if (do_timeseries_output &&
			next_output_time < next_request_time &&
			(transporter_event_queue.empty() || next_output_time < transporter_event_queue.top().first) &&
			(!do_measurement || next_output_time < next_measurement_time)
			)
		{
			event_time = next_output_time;
			output(event_time);

			next_output_time += output_time_step;
		}
		//if the next event is a measurement event (and measurement is enabled)
		else if (do_measurement &&
			next_measurement_time < next_request_time &&
			(transporter_event_queue.empty() || next_measurement_time < transporter_event_queue.top().first)
			)
		{
			event_time = next_measurement_time;
			measurements.measure_system_status(transporter_list, event_time);

			next_measurement_time += measurement_time_step;
		}
		//if the next event is a new_request event
		else if (transporter_event_queue.empty() || next_request_time < transporter_event_queue.top().first)
		{
			ULL event_transporter_index;
			double next_transporter_event;

			ULL request_origin;
			ULL request_destination;

			offer current_offer;
			offer current_best_offer;

			event_time = next_request_time;
			++total_requests;
			request_origin = request_list.begin()->second.first;
			request_destination = request_list.begin()->second.second;

			current_best_offer = offer();
			//find the best offer for the request
			for (transporter& t : transporter_list)
			{
				current_offer = t.best_offer(request_origin, request_destination, event_time, network, current_best_offer);
				if (current_offer.is_better_offer)
				{
					assert(current_offer.dropoff_time <= current_best_offer.dropoff_time);
					current_best_offer = current_offer;
				}
			}

			//assign the request to the best transporter and update the events
			event_transporter_index = current_best_offer.transporter_index;
			next_transporter_event = transporter_list[event_transporter_index].assign_customer(
				event_time,
				customer(request_origin, request_destination, event_time, network, transporter_list[event_transporter_index], current_best_offer),
				current_best_offer,
				network
			);
			if (next_transporter_event >= event_time)
				transporter_event_queue.push(std::make_pair(next_transporter_event, event_transporter_index));

			//update event queue with the next request
			//erase request from the request list
			request_list.erase(request_list.begin());
			if (!request_list.empty())
				next_request_time = request_list.begin()->first;
			else	//if no further request, set request time to a larger time (simulation stops here anyways)
				next_request_time = event_time + std::numeric_limits<double>::max() / 2;

		}
		else {	//the next event is a bus event (bus arriving at a node along its route)

			ULL event_transporter_index;
			double next_transporter_event;

			event_time = transporter_event_queue.top().first;
			event_transporter_index = transporter_event_queue.top().second;
			transporter_event_queue.pop();

			//execute event and handle the next event of the transporter (if any)
			next_transporter_event = transporter_list[event_transporter_index].execute_event(event_time, network, measurements, total_serviced_requests, do_measurement);

			//update event queue with the next event of the transporter
			if (next_transporter_event >= event_time)
				transporter_event_queue.push(std::make_pair(next_transporter_event, event_transporter_index));
		}

		time = event_time;
	}

	//if the simulation is continued with random requests, the next request happens immediately
	next_request_time = time;
}
