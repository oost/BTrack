
class A {};

template <typename I> class B : public A {
public:
  void fn() {}

protected:
  double b_;
};

// template <typename I, typename O>
template <typename O> class C : public B<O> {
public:
  void fn2() {
    this->fn();
    this->b_;
  }
};

int main() {
  C<double> c;
  c.fn2();
}