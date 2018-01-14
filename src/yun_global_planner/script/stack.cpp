#include <vector>
#include <iostream>
#include <stack>

using namespace std;

int main()
{
	stack<int> trace;
	
	
	for(int i = 0; i < 5; i++)
		trace.push(i);
		
	for(int i = 0; i < 5; i++)
		std::cout << trace.top() << std::endl;
	
	return 0;
}
