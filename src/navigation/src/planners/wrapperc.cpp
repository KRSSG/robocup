#include "planners/mergescurve.h"
#include <vector>

/*
	C wrappers for C++ classes:
		* vector<obstacle>
		* Vector2D
		* MergeSCurve
*/

using namespace std;
using namespace Navigation;

extern "C"{

	vector<obstacle>* _vector_obstaclep_new(void){
		return new vector<obstacle>();
	}
	void _vector_obstaclep_delete(vector<obstacle>* v){
		delete v;
	}
	int _vector_obstaclep_size(vector<obstacle>* v){
		return v->size();
	}
	obstacle* _vector_obstaclep_get(vector<obstacle>* v, int pos){
		return &(v->operator[](pos));
	}
	void _vector_obstaclep_push_back(vector<obstacle>* v, obstacle* ob){
		v->push_back(*ob);
	}

	MergeSCurve* _MergeSCurvep_new(void){
		return new MergeSCurve();
	}
	void _MergeSCurvep_delete(MergeSCurve* mc){
		delete mc;
	}
	bool _MergeSCurvep_plan( MergeSCurve* mc,
				 const Vector2D<int>& initial,
				 const Vector2D<int>& final,
				 Vector2D<int>* pt1,
				 Vector2D<int>* pt2,
				 vector<obstacle>* obs,
				 int obstacle_count,
				 int current_id,
				 bool teamBlue	){
		return mc->plan(initial, final, pt1, pt2, *obs, obstacle_count, current_id, teamBlue);
	}

		//Just in case Vector2D python fails
	
	/*Vector2D<int>* _Vector2D_new(void){
		return new Vector2D<int>();
	}
	void _Vector_2D_delete(Vector2D<int>* v){
		delete v;
	}

	void _Vector2D_set(Vector2D<int>* v, int x, int y){
		v->x = x;
		v->y = y;
	}

	int _Vector2D_getx(Vector2D<int>* v){
		return v->x;
	}

	int _Vector2D_gety(Vector2D<int>* v){
		return v->y;
	}*/
}