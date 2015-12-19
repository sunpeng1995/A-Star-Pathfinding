#ifndef __MIN_HEAP__
#define __MIN_HEAP__

#include <vector>

template<class T>
class MinHeap
{
public:
	MinHeap();
	~MinHeap();

	//if find data return index, else return -1
	int find(T data);
	int remove(T data);
	int insert(T data);
	T top();
	bool isEmpty();
	void clear();
	void print();
	T get(int i);


protected:
	std::vector<T> m_heap;
	void filter_down(int start, int end);

	//from start up to 0, adjust the heap
	void filter_up(int start);

};

template<class T>
MinHeap<T>::MinHeap()
{

}

template<class T>
MinHeap<T>::~MinHeap()
{

}

template<class T>
int MinHeap<T>::find(T data)
{
	for (int i = 0; i < m_heap.size(); i++) {
		if (data == m_heap[i])
			return i;
	}
	return -1;
}

template<class T>
T MinHeap<T>::get(int i) {
	return m_heap[i];
}

template<class T>
int MinHeap<T>::remove(T data)
{
	int index;
	index = find(data);
	if (index == -1)
		return index;
	m_heap[index] = m_heap[m_heap.size() - 1];
	m_heap.erase(m_heap.end() - 1);
	filter_down(index, m_heap.size() - 1);

	return 0;
}

template<class T>
int MinHeap<T>::insert(T data)
{
	m_heap.push_back(data);
	filter_up(m_heap.size() - 1);
	return 0;
}

template<class T>
void MinHeap<T>::filter_down(int start, int end)
{
	int cindex = start;
	int l = start * 2 + 1;
	T temp = m_heap[cindex];
	while (l <= end)
	{
		if (l < end && m_heap[l] > m_heap[l + 1])
			l++;
		if(temp < m_heap[l])
			break;
		else {
			m_heap[cindex] = m_heap[l];
			cindex = l;
			l = cindex * 2 + 1;
		}
	}
	m_heap[cindex] = temp;
}

template<class T>
void MinHeap<T>::filter_up(int start)
{
	T temp = m_heap[start];
	int index = start;
	int parent = (index - 1) / 2;
	while (index > 0)
	{
		if (temp > m_heap[parent])
			break;
		m_heap[index] = m_heap[parent];
		index = parent;
		parent = (index - 1) / 2;
	}
	m_heap[index] = temp;
}

template<class T>
T MinHeap<T>::top() {
	T _top = m_heap[0];
	remove(_top);
	return _top;
}

template<class T>
bool MinHeap<T>::isEmpty() {
	return m_heap.size() == 0 ? true : false;
}

template<class T>
void MinHeap<T>::print()
{
	for (int i = 0; i < m_heap.size(); i++) {
		std::cout << m_heap[i] << " ";
	}
}

template<class T>
void MinHeap<T>::clear() {
	m_heap.clear();
}

#endif