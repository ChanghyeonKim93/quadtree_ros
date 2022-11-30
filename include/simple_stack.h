#ifndef _SIMPLE_STACK_H_
#define _SIMPLE_STACK_H_
#include <iostream>

template <typename T>
class SimpleStack {
public:	
    SimpleStack() : MAX_SIZE(65536), size_(0), total_access_(0) {
		values_ = (T*)malloc(sizeof(T)*MAX_SIZE); // 8 * max_size
	};

	~SimpleStack() { delete[] values_; }
	void push(const T& value) {
		if (!isFull()){
			*(values_ + size_) = value;
			++size_;
		}
		else throw std::runtime_error("[ERROR]: SimpleStack is full.");
	}
	void pop() {
		if (!empty()) --size_;
		else printf("[WARN]: SimpleStack is empty.\n");
	}
	T top() {
		if (!empty()) return *(values_ + (size_ - 1));
		else throw std::runtime_error("[ERROR]: SimpleStack is something wrong");
	}
    T topAndPop(){
        if (!empty()) {
            return *(values_ + (--size_));
        }
		else throw std::runtime_error("[ERROR]: SimpleStack is something wrong");
    };
	inline bool empty() { return (size_ < 1); }
	// bool empty() { 
	// 	if(size_ < 1 ) return true;
	// 	else return false;
	// }
	inline int size() {return size_; };
	inline bool isFull() { return (size_ == MAX_SIZE); }
	inline void clear() { size_ = 0; total_access_ = 0;}

// related to the total access
public:
    uint32_t getTotalAccess() const{ return total_access_; };
    void resetTotalAccess() { total_access_ = 0; };
    void addTotalAccess() {++total_access_;};

private:
    uint32_t size_;
	uint32_t MAX_SIZE;
	uint32_t total_access_;
	T* values_; 
};
#endif