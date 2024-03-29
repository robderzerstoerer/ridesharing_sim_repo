#include "measurement_collector.h"

measurement_collector::measurement_collector(traffic_network& param_network) : network(param_network)
{

}

measurement_collector::~measurement_collector()
{
	//dtor
}

void measurement_collector::measure_request(customer& c, transporter& t)
{
	new_measurement(wait_time, c.get_pickup_time() - c.get_request_time());
	new_measurement(drive_time, c.get_dropoff_time() - c.get_pickup_time());
	new_measurement(delay_time, c.get_dropoff_time() - c.get_pickup_time() - network.get_network_distance(c.get_origin(), c.get_destination()) / t.get_velocity());

	if (abs(c.get_dropoff_time() - c.get_pickup_time() - network.get_network_distance(c.get_origin(), c.get_destination()) / t.get_velocity()) > 10 * MACRO_EPSILON)
		new_measurement(fraction_of_delayed_trips, 1);
	else
		new_measurement(fraction_of_delayed_trips, 0);
}

void measurement_collector::measure_trip(std::pair<ULL, double> last_stop, std::pair<ULL, double> current_stop)
{
	new_measurement(drive_distance_between_stops, network.get_network_distance(last_stop.first, current_stop.first));
	new_measurement(drive_time_between_stops, current_stop.second - last_stop.second);
}

//void measurement_collector::measure_stops(...)
//{
//	new_measurement(drive_time_between_stops, c.get_pickup_time() - c.get_request_time());
//	new_measurement(drive_distance_between_stops, c.get_pickup_time() - c.get_request_time());
//}

void measurement_collector::measure_system_status(std::vector<transporter>& transporter_list, double time)
{
	ULL idle_transporters = 0;
	for (transporter& t : transporter_list)
	{
		new_measurement(occupancy, t.get_occupancy());
		new_measurement(scheduled_customers, t.get_number_of_scheduled_customers());
		new_measurement(number_of_planned_stops, t.get_number_of_planned_stops());
		new_measurement(planned_time_horizon, t.get_planned_time_horizon(time));
		if (t.is_idle())
			++idle_transporters;
	}
	new_measurement(number_of_idle_transporters, idle_transporters);
}

void measurement_collector::reset()
{
	wait_time.reset();
	drive_time.reset();
	delay_time.reset();
	fraction_of_delayed_trips.reset();

	drive_time_between_stops.reset();
	drive_distance_between_stops.reset();

	occupancy.reset();
	scheduled_customers.reset();
	number_of_planned_stops.reset();
	planned_time_horizon.reset();
	number_of_idle_transporters.reset();

}

void measurement_collector::print(std::ofstream& out, bool readable)
{
	if (!readable)
	{
		wait_time.print(out);
		drive_time.print(out);
		delay_time.print(out);
		fraction_of_delayed_trips.print(out);

		drive_time_between_stops.print(out);
		drive_distance_between_stops.print(out);

		occupancy.print(out);
		scheduled_customers.print(out);
		number_of_planned_stops.print(out);
		planned_time_horizon.print(out);
		number_of_idle_transporters.print(out);

		out << std::endl;
	}
	else
	{
		out << "MEASUREMENTS" << std::endl << std::endl;

		out << "wait_time: " << std::endl;
		wait_time.print(out);
		out << std::endl;

		out << "drive_time: " << std::endl;
		drive_time.print(out);
		out << std::endl;

		out << "delay_time: " << std::endl;
		delay_time.print(out);
		out << std::endl;

		out << "fraction_of_delayed_trips: " << std::endl;
		fraction_of_delayed_trips.print(out);
		out << std::endl;

		out << "drive_time_between_stops: " << std::endl;
		drive_time_between_stops.print(out);
		out << std::endl;

		out << "drive_distance_between_stops: " << std::endl;
		drive_distance_between_stops.print(out);
		out << std::endl;

		out << "occupancy:" << std::endl;
		occupancy.print(out);
		out << std::endl;

		out << "scheduled_customers: " << std::endl;
		scheduled_customers.print(out);
		out << std::endl;

		out << "number_of_planned_stops: " << std::endl;
		number_of_planned_stops.print(out);
		out << std::endl;

		out << "planned_time_horizon: " << std::endl;
		planned_time_horizon.print(out);
		out << std::endl;

		out << "number_of_idle_transporters: " << std::endl;
		number_of_idle_transporters.print(out);
		out << std::endl;

		out << std::endl;
	}
}

void measurement_collector::new_measurement(measure& m, long double val)
{
	long double delta1, delta2;

	m.n += 1;
	delta1 = val - m.av;
	m.av += delta1 / m.n;
	delta2 = val - m.av;
	m.stddev += delta1 * delta2;
}
