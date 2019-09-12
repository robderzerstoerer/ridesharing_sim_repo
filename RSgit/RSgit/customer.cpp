#include "customer.h"

customer::customer(ULL param_origin, ULL param_destination, double param_time, traffic_network& n, transporter& t, offer& o) :
	origin(param_origin), destination(param_destination), request_time(param_time), offer_pickup_time(o.pickup_time), offer_dropoff_time(o.dropoff_time), pickup_time(-1), dropoff_time(-1)
{
	//set allowed delay (relative to remaining time until promised stop)
	//values < 1 means no delay at all
	//set to e.g. 1.10 to allow a 10% delay relative to (promised - current) time
	allowed_pickup_delay = 0;
	allowed_dropoff_delay = 0;
}

customer::~customer()
{
	//dtor
}
