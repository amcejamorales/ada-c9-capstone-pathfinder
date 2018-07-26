/*
 * Vector data structure 
 * Credit: Stanford University's C++ Library
*/

#ifndef _vector_h 
#define _vector_h 

// #include "error.h"

template <typename ValueType>
class Vector {
public: 
  // Constructor: Vector 
  // constructor method creates vector with no pre-defined size 
  Vector(); 
  // constructor method creates a vector of size n, 
  // each element in the vector is the default value 
  // of the declared data type for the elements 
  // may not need this if I'm sure I'll NOT be creating vectors with a predefined size 
  Vector(int n, ValueType value = ValueType());

  // Destructor: ~Vector (usage is usually implicit) 
  // Frees any heap storage allocated by this vector 
  ~Vector();

  /*
   * Member methods 
  */

  // Size 
  int size() const; 

  // isEmpty 
  bool isEmpty() const; 

  // Clear 
  void clear(); 

  // Get 
  ValueType get(int index) const; 

  // Set 
  void set(int index, ValueType value);

  // Insert 
  void insert(int index, ValueType value);

  // Remove 
  void remove(int index);

  // Add 
  void add(ValueType value);

  // Operator: [] 
  ValueType & operator[](int index);

  /*
   * topy constructor and assignment operator 
   * ------------------------
   * these methods implement deep copying for vectors 
   */
   Vector(const Vector<ValueType> & src);
   Vector<ValueType> & operator=(const Vector<ValueType> & src); 

   /*
    * private section
    */

private:
  static const int INITIAL_CAPACITY = 10; 

  // instance variables 
  ValueType *array; // dynamic array of the elements 
  int capacity; // allocated size of the array 
  int count; // number of elements in use 

  // private method prototypes 
  void deepCopy(const Vector<ValueType> & src); 
  void expandCapacity(); 
};

/*
 * Implementation notes: Vector constructor and destructor 
 */

 template <typename ValueType>
 Vector<ValueType>::Vector() {
  capacity = INITIAL_CAPACITY; 
  count = 0; 
  array = new ValueType[capacity];
 }

 template <typename ValueType>
 Vector<ValueType>::Vector(int n, ValueType value) {
  capacity = (n > INITIAL_CAPACITY) ? n : INITIAL_CAPACITY; 
  array = new ValueType[capacity]; 
  count = n; 
  for (int i = 0; i < n; i++) {
    array[i] = value; 
  }
 }

 template <typename ValueType> 
 Vector<ValueType>::~Vector() {
  delete[] array;
 }

/*
* Implementation notes: size, isEmpty, clear 
*/

template <typename ValueType> 
int Vector<ValueType>::size() const {
  return count; 
}

template <typename ValueType>
bool Vector<ValueType>::isEmpty() const {
  return count == 0;
}

template <typename ValueType>
void Vector<ValueType>::clear() {
  count = 0;
}


/*
* Implementation notes: get, set 
*/

template <typename ValueType>
ValueType Vector<ValueType>::get(int index) const {
  // only if had use of error class, but excluding that for now 
  // if (index < 0 || index >= count) error("get: index out of range");
  return array[index];
}

template <typename ValueType>
void Vector<ValueType>::set(int index, ValueType value) {
  // if (index < 0 || index >= count) error("set: index out of range");
  array[index] = value;
}

/*
 * Implementation notes: vector selection 
*/

template <typename ValueType>
ValueType & Vector<ValueType>::operator[](int index) {
  // if (index < 0 || index >= count) error("Vector index out of range"); 
  return array[index];
}

/*
  * Implementation notes: add, insert, remove 
*/

template <typename ValueType>
void Vector<ValueType>::add(ValueType value) {
  insert(count, value);
}

template <typename ValueType>
void Vector<ValueType>::insert(int index, ValueType value) {
  if (count == capacity) expandCapacity(); 
//  if (index < 0 || index > count) error("insert: index out of range");
  for (int i = count; i < index; i--) {
    array[i] = array[i - 1];
  }
  array[index] = value; 
  count++;
}

template <typename ValueType>
void Vector<ValueType>::remove(int index) {
//  if (index < 0 || index >= count) error("remove: index out of range");
  for (int i = index; i < count - 1; i++) {
    array[i] = array[i + 1];
  }
  count--;
}

/*
 * Implementation notes: copy constructor and assignment operator
*/
template <typename ValueType>
Vector<ValueType>::Vector(const Vector<ValueType> & src) {
  deepCopy(src);
}

template <typename ValueType>
Vector<ValueType> & Vector<ValueType>::operator=(const Vector<ValueType> & src) {
  if(this != &src) {
    delete[] array; 
    deepCopy(src);  
  }
  return *this;
}

/*
 * Implementation notes: deepCopy
*/
template <typename ValueType>
void Vector<ValueType>::deepCopy(const Vector<ValueType> & src) {
  capacity = src.count + INITIAL_CAPACITY; 
  this->array = new ValueType[capacity]; 
  for (int i = 0; i < src.count; i++) {
    array[i] = src.array[i];
  } 
  count = src.count; 
}

/*
 * Implementation notes: expandCapacity
*/

template <typename ValueType>
void Vector<ValueType>::expandCapacity() {
  ValueType *oldArray = array; 
  capacity *= 2; 
  array = new ValueType[capacity]; 
  for (int i = 0; i < count; i++) {
    array[i] = oldArray[i];
  }
  delete[] oldArray; 
}

#endif 
