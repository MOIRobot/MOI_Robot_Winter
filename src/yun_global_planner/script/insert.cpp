#include <iostream>
#include <vector>

using namespace std;

typedef struct Waypoint{
	float dx;
	float dy;
}Waypoint;

void ch_len(float* len){

	*len = 10;

}

int main(int argc, char* argv[])
{

	Waypoint s0, s1;
	s0.dx = 10;
	s0.dy = 12;
	
	s1 = s0;
	
	cout << "s0 : " << s0.dx << " " << s0.dy << endl;
	cout << "s1 : " << s1.dx << " " << s1.dy << endl;
	
	
	vector<int> v1;
	float length = 0;
	
	v1.push_back(2);
	v1.push_back(3);
	v1.push_back(4);
	
	v1.insert(v1.begin(), 1);
	v1.insert(v1.end(), 5);
	
	for(vector<int>::iterator it = v1.begin(); it != v1.end(); it++){
		cout << *it << endl;
	}
	
	ch_len(&length);
	
	cout << "length :" << length << endl;
	


	return 0;
}



















