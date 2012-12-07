#ifndef ___PID___
#define ___PID___

template <class T>
class  PIDControl
{
	private:
		//PID Variables, should really make a PID class
		float sum;
		float diff;
		float sumnation[SUM_SIZE-1];
		int sumposition;

	public:
		T* input, output;
		T desired;
		float P,I,D;
		PIDControl(T*, T*, T,T,T, T);
		T operate();
};


#endif
