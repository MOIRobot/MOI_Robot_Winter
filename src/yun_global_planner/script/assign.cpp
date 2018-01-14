#include <vector>
#include <iostream>
#include <algorithm>

int main()
{
	using namespace std;
	vector<int> v1, v2, v3;
	vector<int>::iterator iter;
	//reserve避免不必要的重新分配 
	v1.reserve(10);
	v2.reserve(10);
	v3.reserve(10);
	for(int i = 0 ; i < 10; i++){
		v1.push_back(i);
		//当当前分配的内存不够时，容器会自动分配比现在大一倍的内存。
		cout << "Capacity of v1 : " << v1.capacity() << endl;
	}
	cout << "Size of v1 : " << v1.size() << endl;
	cout << "Capacity of v1 : " << v1.capacity() << endl;
	
	cout << "v1 = :";
	for(iter = v1.begin(); iter != v1.end(); iter++)
		cout << *iter << " ";
	cout << endl;
	
	//copy v1[0] - v1[4] to v2
	v2.assign(v1.begin(), v1.end() - 5);
	cout << "v2 = :";
	for(iter = v2.begin(); iter!=v2.end(); iter++)
		cout << *iter << " ";
	cout << endl;
	
	//copy 10th 3 to v3
	v3.assign(10, 3);
	cout << "v3 = :";
	for(iter = v3.begin(); iter!=v3.end(); iter++)
		cout << *iter << " ";
	cout << endl;

	reverse(v1.begin(), v1.end()-5);
	cout << "reverse v1 = :";
	for(iter = v1.begin(); iter!=v1.end(); iter++)
		cout << *iter << " ";
	cout << endl;
	
	vector<bool> isInS;
	isInS.reserve(0);
	cout << "sizeof the isInS: " << isInS.size() << endl;
	
	return 0;
}














