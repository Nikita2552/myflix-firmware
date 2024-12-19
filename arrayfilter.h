#ifndef AFFAY_F
#define AFFAY_F

#include <vector>

using namespace std;

template <typename T>
class ArrayFilter
{
public:
	ArrayFilter(unsigned int pa) : parts(pa), primarypart(0) {};
	T update(float tovec);
	
private:
 T primarypart;
 vector<T> vec;
 unsigned int parts;
};

template <typename T>
T ArrayFilter<T>::update(float tovec)
{
	if (vec.size() < parts)
	{
		vec.push_back(tovec);
	}
	else
	{		
		for (int i = 0; i < vec.size(); i++)
		{
			primarypart += vec[i];			
		}
		primarypart /= parts;
		vec.clear();
	}
	
	return primarypart;
}


#endif
