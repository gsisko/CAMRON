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
        sum = 0;
        diff = 0;
}


template <class T>
T PIDControl<T>::operate(T *input, T* output)
{
	diff = sumnation[sumposition];
        sumposition += 1;
	if(sumposition >= SUM_SIZE)
		sumposition = 0;
        sumnation[sumposition] = desired - *input;
        diff = sumnation[sumposition] - diff; 
	int i = SUM_SIZE;
        sum = 0;
        for(i = 0; i < SUM_SIZE; i++)
          {
            //Serial.println(sumnation[i]);
            sum += sumnation[i];
          }
        
        Serial.print(sum);
        Serial.print("  .   ");
	return *output = (sumnation[sumposition]*P)+(sum*I)-(diff*D);
        
}

