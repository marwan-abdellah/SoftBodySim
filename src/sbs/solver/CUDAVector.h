#ifndef SBS_SOLVER_CUDA_VECTOR_H_
#define SBS_SOLVER_CUDA_VECTOR_H_

#include "common.h"

template <typename T>
class CUDAVector {
public:
	CUDAVector(void);
	CUDAVector(const CUDAVector &v);
	CUDAVector(size_t n);
	~CUDAVector(void);
	void push_back(T &data);
	void push_back(T *data, size_t n);
	size_t size(void) { return mSize; }
	void clear(void);
	void resize(size_t n);
	T* data(void) { return mPtr; }
private:
	bool ReallocInternal(size_t n);
	T *mPtr;
	size_t mSize;
	size_t mAlloc;
};

template <typename T>
CUDAVector<T>::CUDAVector(void) :
	mPtr(0),
	mSize(0),
	mAlloc(0)
{
}

template <typename T>
CUDAVector<T>::CUDAVector(size_t n) :
	mPtr(0),
	mSize(0)
{
	ReallocInternal(n);
}

template <typename T>
CUDAVector<T>::~CUDAVector(void)
{
	clear();
}

template <typename T>
void CUDAVector<T>::push_back(T &data)
{
	if (mSize >= mAlloc)
		if (!ReallocInternal(mAlloc * 2)) return;
	cudaMemcpy(mPtr + mSize, &data, sizeof(T), cudaMemcpyHostToDevice);
	mSize++;
}

template <typename T>
void CUDAVector<T>::push_back(T *data, size_t count)
{
	if (mSize + count >= mAlloc)
		if (!ReallocInternal(mSize + count)) return;
	cudaMemcpy(mPtr + mSize, data, count * sizeof(T), cudaMemcpyHostToDevice);
	mSize += count;
}

template <typename T>
bool CUDAVector<T>::ReallocInternal(size_t n)
{
	cudaError_t err;
	T *tmp;
	err = cudaMalloc(&tmp, n * sizeof(T));
	if (err != cudaSuccess) return false;
	if (mSize > 0) {
		int size = mSize > n ? n : mSize; // clip if current buffer is lesser
		err = cudaMemcpy(tmp, mPtr, sizeof(T) * size , cudaMemcpyDeviceToDevice); 
		if (err != cudaSuccess) {
			ERR("Failed to copy data on device!");
			return false;
		}
	}
	if (mPtr)
		cudaFree(mPtr);
	mPtr = tmp;
	mAlloc = n;
	return true;
}

template <typename T>
void CUDAVector<T>::clear(void)
{
	if (mPtr)
		cudaFree(mPtr);
	mSize = mAlloc = 0;
	mPtr = NULL;
}

template <typename T>
void CUDAVector<T>::resize(size_t n)
{
	int size = mSize;
	if (n > mAlloc)
		ReallocInternal(n);
	if (n > size)
		cudaMemset(mPtr + size, 0x0, sizeof(T) * (n - size));
}

#endif
