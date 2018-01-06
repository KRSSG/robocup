#ifndef UTILS
#define UTILS
#include <iostream>
#include <bits/stdc++.h>

namespace Utils
{		
	template <typename T>
	struct Point
	{
		T x;
		T y;
		Point<T> operator+(Point<T> const&rhs)
		{
			Point<T> temp;
			temp.x=this->x + rhs.x;
			temp.y=this->y + rhs.y;
			return temp;
		}
		Point<T> operator-(Point<T> const&rhs)
		{
			Point<T> temp;
			temp.x=this->x - rhs.x;
			temp.y=this->y - rhs.y;
			return temp;
		}
		Point<T> operator=(Point<T> const&rhs)
		{
			x=rhs.x;
			y=rhs.y;
		}
		bool operator==(Point<T> const&rhs)
		{
			if(x==rhs.x && y==rhs.y)
				return true;
			return false;
		}
		bool operator!=(Point<T> const&rhs)
		{
			if(x==rhs.x && y==rhs.y)
				return false;
			return true;
		}
	};
	
}

#endif