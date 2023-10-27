#include <iostream>

#include "scenes_cloth.h"
#include "scenes_deformable_solids.h"
#include "scenes_rigidbodies.h"
#include "scenes_interactions.h"
#include "kobuki.h"
#include "gripper.h"

int main()
{
	// ------------------------- CLOTH -------------------------
	//hanging_cloth();
	//collision_cloth_test();
	//collision_cloth_edge_edge_tests();
	//collision_cloth_parallel_edge_test_rotation();
	//collision_cloth_parallel_edge_test_shear();
	//collision_cloth_parallel_edge_test_slide();
	//cloth_wrap();

	//cloth_friction_slope_test();
	//cloth_friction_corner();


	// ------------------------- RIGID BODIES -------------------------
	//rb_ball_joint();
	//rb_slider();
	//rb_contacts_floor_test();
	//rb_contact_edge_test();
	//laundry();


	// ------------------------- INTERACTIONS -------------------------
	//interaction_cloth_rb();
	//interaction_cloth_rb_bowl();
	laundry_cloth();


	// ------------------------- KOBUKI -------------------------
	//kobuki_test();
	//towel_parametrization();
	//folding_towel();
	//rolling_towel();
	//kobuki_v_towel_suite();


	// ------------------------- GRIPPER -------------------------
	//gripper_box();
	//gripper_cup(5e-3);


	// ------------------------- DEFORMABLE VOLUMETRICS -------------------------
	//andreas_cantilever();
}
