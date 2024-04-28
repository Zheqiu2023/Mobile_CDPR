#pragma once

#include <vector>
#include <memory>

template <typename T>
class Filter
{
  public:
    Filter() = default;
    virtual ~Filter() = default;
    virtual void input(T input_value) = 0;
    virtual T output() = 0;
    virtual void clear() = 0;
};

template <typename T>
class ButterworthFilter : public Filter<T>
{
  public:
    ButterworthFilter(int num_sample, T dt, T cutoff_frequency);
    ~ButterworthFilter();
    void input(T input_value);
    T output();
    void clear();

  private:
    T* mpBuffer_;
    int mCurIdx_;
    int mNumSample_;
    T mDt_;
    T mCutoffFreq_;
    T mValue_;
};

template <typename T>
class DigitalLpFilter : public Filter<T>
{
  public:
    DigitalLpFilter(T w_c, T t_s);
    ~DigitalLpFilter();
    void input(T input_value);
    T output();
    void clear();

  private:
    T Lpf_in_prev_[2];
    T Lpf_out_prev_[2];
    T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
    T lpf_out_;
};

template <typename T>
class MovingAverageFilter : public Filter<T>
{
  public:
    explicit MovingAverageFilter(int num_data);
    ~MovingAverageFilter();
    void input(T input_value);
    T output();
    void clear();

  private:
    T* buffer_;
    int num_data_;
    int idx_;
    T sum_;
};

template <typename T>
class Vector3WithFilter
{
  public:
    Vector3WithFilter(int num_data)
    {
        for (int i = 0; i < 3; i++)
            filter_vector_.push_back(std::make_shared<MovingAverageFilter<T>>(num_data));
    }
    void input(T vector[3])
    {
        for (int i = 0; i < 3; i++)
            filter_vector_[i]->input(vector[i]);
    }
    void clear()
    {
        for (int i = 0; i < 3; i++)
            filter_vector_[i]->clear();
    }
    T x()
    {
        return filter_vector_[0]->output();
    }
    T y()
    {
        return filter_vector_[1]->output();
    }
    T z()
    {
        return filter_vector_[2]->output();
    }

  private:
    std::vector<std::shared_ptr<MovingAverageFilter<T>>> filter_vector_;
};

template <typename T>
class DerivLpFilter : public Filter<T>
{
  public:
    DerivLpFilter(T w_c, T t_s);
    ~DerivLpFilter();
    void input(T input_value);
    T output();
    void clear();

  private:
    T Lpf_in_prev_[2];
    T Lpf_out_prev_[2];
    T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
    T lpf_out_;
};

template <typename T>
class FF01Filter : public Filter<T>
{
  public:
    FF01Filter(float t_s, float w_c);
    virtual ~FF01Filter();
    virtual void input(T input_value);
    virtual T output();
    virtual void clear();

  private:
    T Lpf_in_prev_[2];
    T Lpf_out_prev_[2];
    T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
    T lpf_out_;
};

template <typename T>
class FF02Filter : public Filter<T>
{
  public:
    FF02Filter(float t_s, float w_c);
    ~FF02Filter();
    void input(T input_value);
    T output();
    void clear();

  private:
    T Lpf_in_prev_[2];
    T Lpf_out_prev_[2];
    T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
    T lpf_out_;
};

template <typename T>
class AverageFilter : public Filter<T>
{
  public:
    AverageFilter(T dt, T t_const, T limit);
    ~AverageFilter();
    void input(T input_value);
    T output();
    void clear();

  private:
    T est_value_;
    T dt_;
    T t_const_;
    T limit_;
};

template <typename T>
class RampFilter : public Filter<T>
{
  public:
    RampFilter(T acc, T dt);
    ~RampFilter() = default;
    void input(T input_value);
    void clear();
    void clear(T last_value);
    void setAcc(T acc);  // without clear.
    T output();

  private:
    T last_value_;
    T acc_;
    T dt_;
};

template <typename T>
class OneEuroFilter : public Filter<T>
{
  public:
    OneEuroFilter(double _freq, T _mincutoff, T _beta, T _dcutoff);
    ~OneEuroFilter();
    void input(T input_value);
    T output();
    void clear();

  private:
    double freq;
    bool firsttime;
    T mincutoff, beta, dcutoff;
    T x_prev, dhatxprev, hatxprev;
    T filtered_val;
};

/**
 * @brief 中位均值滤波：删除最大最小元素，再求和取平均值
 * @tparam T
 * @param  vec
 * @return T
 */
template <typename T>
T medianMeanFilter(std::vector<T>& vec)
{
    sort(vec.begin(), vec.end());
    vec.erase(vec.begin());
    vec.pop_back();
    T sum = accumulate(vec.begin(), vec.end(), T(0));
    return sum / vec.size();
};
