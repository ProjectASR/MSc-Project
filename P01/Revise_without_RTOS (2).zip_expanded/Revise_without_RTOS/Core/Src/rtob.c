#include <rtob.h>

void set_rtob(rtob_instance *rtob, float k, float j, float g, float f, float b) {

	rtob->Kt = k;
	rtob->Jm = j;
	rtob->G_dis = g;
	rtob->temp = 0;
	rtob->F = f;
	rtob->B = b;

}

void update_rtob(rtob_instance *rtob, float Ia_ref, float velocity) {

	float T_in = Ia_ref * rtob->Kt + rtob->Jm * rtob->G_dis * velocity - rtob->F
			- rtob->B * velocity;
	rtob->temp = rtob->temp + (T_in - rtob->temp) * rtob->G_dis * dt/1000.0; // /dt

	rtob->T_extern = rtob->temp - rtob->Jm * rtob->G_dis * velocity;

}
