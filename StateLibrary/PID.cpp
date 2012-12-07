#include "PID.h"
template <class T>
PIDControl<T>::PIDControl(T _P,T _I,T _D, T _desired)
{
	
	
        
	desired = _desired;
	P = _P; 
	I = _I; 
	D = _D;
	for(int i=0; i<SUM_SIZE;i++)
	{
		sumnation[i] = 0;
	}
	sumposition = 0;
}


template <class T>
T PIDControl<T>::operate(T *input, T* output)
{
	sumposition += 1;
	if(sumposition > SUM_SIZE)
		sumposition = 0;
	sumnation[sumposition] = desired - *input; 
	diff = sumnation[sumposition] - sumnation[((sumposition - 1) < 0) ?SUM_SIZE : (sumposition - 1)];
	sum += sumnation[sumposition];
	sum -= sumnation[((sumposition+1) < SUM_SIZE)? (sumposition+1) : 0];

	return *output = (sumnation[sumposition]*P)+(sum*I)-(diff*D);
        
}

