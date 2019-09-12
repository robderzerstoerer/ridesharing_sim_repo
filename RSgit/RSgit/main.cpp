#include <cstddef>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdio>
#include <algorithm>
#include <numeric>
#include <queue>
#include <map>
#include <list>
#include <set>
#include <tuple>
#include <vector>
#include <cmath>
#include <random>

#include <cassert>

#include "ridesharing_sim.h"

#ifndef _INTEGER_TYPES
#define ULL uint64_t
#define LL int64_t
#define _INTEGER_TYPES
#endif

#ifndef _EPSILON
#define MACRO_EPSILON 0.000001
#define _EPSILON
#endif

constexpr double pi = 3.14159265358979323846;

//ULL and double: 64-bit variables for everything

//Simulation of taxi system with arbitrary networks (needs to be strongly connected) and taxis with different service types
//everything is independent of the destination of a request (constant maximal waiting time for all requests, indiscriminate service of all requests)
int main(int argc, char* argv[])
{
	std::string topology = "torus"; // new comm
	ULL number_of_buses = 100;
	ULL number_of_nodes = 25;
	double normalized_request_rate = 7.5;

	std::stringstream filename("");
	filename << topology + "_N_25__B_100__x_7.5.dat";
	std::ofstream out(filename.str().c_str());

	//initialize simulation class
	ridesharing_sim sim(number_of_nodes, number_of_buses, 0);

	//create topologies
	//add links for each node in the network
	for (unsigned int i = 0; i < number_of_nodes; ++i)
	{
		if (topology == "two_nodes")
		{
			assert(number_of_nodes == 2);

			if (i == 0)
			{
				sim.network.add_link(0, 1, 1);
				sim.network.add_link(1, 0, 1);
			}
		}

		if (topology == "ring")
		{
			//create a path-graph (a line)
			if (i < number_of_nodes - 1)
			{
				sim.network.add_link(i, i + 1, 1);
				sim.network.add_link(i + 1, i, 1);
			}

			//close the ring
			if (i == number_of_nodes - 1)
			{
				sim.network.add_link(0, number_of_nodes - 1, 1);
				sim.network.add_link(number_of_nodes - 1, 0, 1);
			}

		}

		if (topology == "torus")
		{
			//create a square lattice first
			unsigned int L = sqrt(number_of_nodes);
			if (i%L != L - 1)
			{
				sim.network.add_link(i, i + 1, 1);
				sim.network.add_link(i + 1, i, 1);
			}
			if (i < L*(L - 1))
			{
				sim.network.add_link(i, i + L, 1);
				sim.network.add_link(i + L, i, 1);
			}

			//add periodic boundaries
			if (i%L == L - 1)
			{
				sim.network.add_link(i, i + 1 - L, 1);
				sim.network.add_link(i + 1 - L, i, 1);
			}
			if (i >= L*(L - 1))
			{
				sim.network.add_link(i, (i + L) % number_of_nodes, 1);
				sim.network.add_link((i + L) % number_of_nodes, i, 1);
			}
		}
	}

	//pre-calculate distance matrix (to find shortest paths later)
	sim.network.create_distances();

	//set probability distribution for origin and destination of requests
	//requests are uncorrelated from one random node to another random node
	//note: this currently allows requests from a node to itself (this should probably be changed due to mathematical problems when trying to calculate with this in extreme cases)
	sim.network.set_origin_probabilities();			//default: uniform
	sim.network.set_destination_probabilities();	//default: uniform
													//recompute mean distance with respect to request distribution
	sim.network.recalc_mean_distances();

	//	std::cout << sim.network.get_mean_pickup_distance() << '\t' << sim.network.get_mean_dropoff_distance() << std::endl;

	//set up (reset) all buses in the network
	for (ULL b = 0; b < number_of_buses; ++b)
	{
		sim.transporter_list[b].reset(b, sim.network.generate_request().second, 0);
	}


	//create requests to equally distribute buses on the topology (important especially in small topologies with many buses)
	std::list< std::pair< double, std::pair<ULL, ULL> > > request_list;
	if (topology == "two_nodes")
	{
		//equally spaced requests from each node to the other
		for (double t = 0; t < 1.0; t += 2.0 / number_of_buses)
		{
			request_list.push_back(std::make_pair(t, std::make_pair(0, 1)));
			request_list.push_back(std::make_pair(t, std::make_pair(1, 0)));
		}
	}
	if (topology == "ring")
	{
		//equally spaced requests from each node to the other
		for (double t = 0; t < 1.0; t += 2.0 * number_of_nodes / (1.0 * number_of_buses))
		{
			for (ULL i = 0; i < number_of_nodes; ++i)
			{
				request_list.push_back(std::make_pair(t, std::make_pair(i, (i + 1) % number_of_nodes)));
				request_list.push_back(std::make_pair(t, std::make_pair((i + 1) % number_of_nodes, i)));
			}
		}
	}
	if (topology == "torus")
	{
		if (number_of_buses > 0.25 * number_of_nodes)
		{
			for (double t = 0; t < 1.0; t += (4.0 * number_of_nodes) / (number_of_buses))
			{
				unsigned int L = sqrt(number_of_nodes);

				//equally spaced requests from each node to its neighbors
				for (unsigned int i = 0; i < number_of_nodes; ++i)
				{
					if (i % L == 0)
						request_list.push_back(std::make_pair(t, std::make_pair(i, i + L - 1)));
					else
						request_list.push_back(std::make_pair(t, std::make_pair(i, i - 1)));

					if (i % L == L - 1)
						request_list.push_back(std::make_pair(t, std::make_pair(i, i + 1 - L)));
					else
						request_list.push_back(std::make_pair(t, std::make_pair(i, i + 1)));

					if (i + L >= number_of_nodes)
						request_list.push_back(std::make_pair(t, std::make_pair(i, i + L - number_of_nodes)));
					else
						request_list.push_back(std::make_pair(t, std::make_pair(i, i + L)));

					if (i < L)
						request_list.push_back(std::make_pair(t, std::make_pair(i, i + number_of_nodes - L)));
					else
						request_list.push_back(std::make_pair(t, std::make_pair(i, i - L)));
				}
			}
		}
	}


	//simulate the requests to realize equal distribution of buses
	sim.run_sim_request_list(request_list);

	//set the request rate for random requests
	sim.set_normalized_request_rate(normalized_request_rate);
	//equilibrate simulation for 10 requests per bus (at least 1000 requests) to obtain a "correct" initial condition
	//this may not be long enough if the initial request list was too long or the network is large, some try-and-error may be needed here
	sim.run_sim_requests(std::max((ULL)1000, 10 * number_of_buses));
	//turn on measurements with a given step size, measure every (number of buses) requests for a total of ~ 100 measurements
	sim.enable_measurements((1.0*number_of_buses) / sim.request_rate);
	//simulate (and measure) for 100 requests per bus (at least 10000 requests)
	sim.run_sim_requests(std::max((ULL)10000, 100 * number_of_buses));

	//output results
	sim.print_params(out);
	sim.print_measurements(out);

	out.close();

	return(0);
}

