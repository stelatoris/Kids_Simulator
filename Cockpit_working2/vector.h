#pragma once

// an almost real vector of Ts:
template<typename T>
class vector { // read “for all types T” (just like in math)
  int sz; // the size
  T* elem; // a pointer to the elements
  int space; // size + free space
public:
  vector() : sz{ 0 }, elem{ nullptr }, space{ 0 } { }
  explicit vector(int s) :sz{ s }, elem{ new T[s] }, space{ s }
  {
    for (int i = 0; i < sz; ++i) elem[i] = 0; // elements are initialized
  }
  vector(const vector& a)                 // copy constructor
        : sz{a.sz}, elem{new T[a.sz]}, space{a.sz}
    {
        for (int i = 0; i < a.sz; ++i)
            elem[i] = a.elem[i];
    }

  vector& operator=(const vector&); // copy assignment
  vector(vector&&); // move constructor
  vector& operator=(vector&&); // move assignment
  ~vector() { delete[] elem; } // destructor
  T& at(int n);
  const T& at(int n) const;

  T& operator[] (int n) { return elem[n]; } // access: return reference
  const T& operator[] (int n) const { return elem[n]; }
  int size() const { return sz; } // the current size
  int capacity() const { return space; }
  void resize(int newsize, T val = T()); // growth
  void push_back(const T& val);
  void reserve(int newalloc);
};

template<typename T>
vector<T>& vector<T>::operator=(const vector& a)
{
  if (this == &a) return *this; // self-assignment, no work needed
  if (a.sz <= space) { // enough space, no need for new allocation
    for (int i = 0; i < a.sz; ++i) elem[i]=a.elem[i]; // copy elements
    sz = a.sz;
    return *this;
  }
  T* p =  new T[a.sz];  // allocate new space
  for (int i = 0; i < a.sz; ++i) p[i]=a.elem[i];  // copy elements
  delete[] elem;  // deallocate old space
  space = sz = a.sz; // set new size
  elem = p; // set new elements
  return *this; // return a self-reference
}

template<typename T>
vector<T>::vector(vector&& a)
  :sz{ a.sz }, elem{ a.elem } // copy a’s elem and sz
{
  a.sz = 0; // make a the empty vector
  a.elem = nullptr;
}

template<typename T>
vector<T>& vector<T>::operator=(vector&& a) // move a to this vector
{
  delete[] elem; // deallocate old space
  elem = a.elem; // copy a’s elem and sz
  sz = a.sz;
  a.elem = nullptr; // make a the empty vector
  a.sz = 0;
  return *this; // return a self-reference (see §17.10)
}

template<typename T>
void vector<T>::reserve(int newalloc)
{
  if (newalloc <= space) return;
  T* p = new T[newalloc];
  
  for (int i = 0; i < sz; ++i) p[i] = elem[i]; // copy old elements
  delete[] elem;
  elem = p;
  space = newalloc;
}

template<typename T>
void vector<T>::resize(int newsize, T val)
// make the vector have newsize elements
// initialize each new element with the default value 0.0
{
  reserve(newsize);
  for (int i = sz; i < newsize; ++i) elem[i]=0;
  sz = newsize;
}

template<typename T>
void vector<T>::push_back(const T& val)
// increase vector size by one; initialize the new element with d
{
  if (space == 0)
    reserve(8); // start with space for 8 elements
  else if (sz == space)
    reserve(2 * space); // get more space
  elem[sz]=val;
  ++sz;
}

template<typename T>
T& vector<T>::at(int n)
{
  if (n < 0 || sz <= n) {}
  return elem[n];
}
