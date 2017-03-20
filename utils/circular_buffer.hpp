#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#define WRAP_FORWARD(_i_,_s_) ((((_i_)+1) == (_s_))?(0):((_i_)+1))
#define WRAP_BACKWARD(_i_,_s_) (((_i_) == 0)?((_s_)-1):((_i_)-1))

template<typename T, uint32_t _size>
class CircularBuffer
{
private:
    T m_elems[_size];
    uint32_t head, tail;
public:
    CircularBuffer()
    {
        head = tail = 0;
    }

    void push(const T& data)
    {
        if(head == WRAP_BACKWARD(tail, _size))
            tail = WRAP_FORWARD(tail, _size);

        m_elems[head] = data;

        head = WRAP_FORWARD(head, _size);
    }

    bool pop()
    {
        if(tail == head)
            return false;

        tail = WRAP_FORWARD(tail, _size);

        return true;
    }

    T& front()
    {
        return m_elems[tail];
    }

    bool empty()
    {
        return (head == tail); 
    }
};

#undef WRAP_FORWARD
#undef WRAP_BACKWARD

#endif // CIRCULAR_BUFFER_HPP
