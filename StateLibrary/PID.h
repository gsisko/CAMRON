#ifndef ___PID___
#define ___PID___


template <class T>
class  PIDControl
{
	private:
		//PID Variables, should really make a PID class
		T sum;
		T diff;
		T sumnation[SUM_SIZE-1];
		int sumposition;

	public:
		T *output;
		T desired;
		T P,I,D;
                PIDControl(T,T,T, T);
		T operate(T*, T*);
                T* checkLoc();
};


#endif
