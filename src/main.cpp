#include <iostream>
#include <string>
#include "vrep_bridge.h"

#include "controller.h"

#include "linux_terminal_tool.h"
#define MODE(X,Y) case X: ac.setMode(Y); break;


using namespace std;

int main()
{
	VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
	// VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
	const double hz = 1000 ;
	ArmController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		ac.readData(vb.getPosition(), vb.getVelocity());
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			ac.readData(vb.getPosition(), vb.getVelocity());
			cout << "Initial q: " << vb.getPosition().transpose() << endl;
			is_first = false;
			ac.initPosition();
		}

		if (kbhit())
		{
			int key = getchar();
			switch (key)
			{
				// Implement with user input
      MODE('i', "joint_ctrl_init")
			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			vb.setDesiredPosition(ac.getDesiredPosition());
			vb.setDesiredTorque(ac.getDesiredTorque());
		
			vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}
