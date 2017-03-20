#ifndef CAMERA_UTILS_H
#define CAMERA_UTILS_H

#define MAX_EDGES (16)
#define THRESHOLD (0.4f)
#define MAX_LINE_WIDTH (40)
#define MIN_LINE_WIDTH (4)
#define EDGE_CLIP_COUNT (5)

namespace CameraUtils
{

template<class T, size_t buffer_size>
struct LinecamProcessor
{

    // Used to rank edges
    struct LineEdge_t
    {
        T pos;
        T delta;

        LineEdge_t()
        {
            pos = 0;
            delta = 0;
        }
        
        LineEdge_t(T _pos, T _delta)
        {
            pos = _pos;
            delta = _delta;
        }

        static T distance(const LineEdge_t& a, const LineEdge_t& b)
        {
            return abs(a.pos - b.pos);
        }
    };

    static float m_centerpos;
        // Allocate space to store the derivative
        static T derivative_buffer[buffer_size - 1];

    static void init()
    {
        m_centerpos = 0.0f;
    }

    static void update(volatile const T* buffer)
    {

        // Allocate some space to store the edge objects
        LineEdge_t edges_buffer[MAX_EDGES];
        size_t edges_buffer_top = 0;

        // Calculate the maximum derivative (for thresholding)
        T max_derivative = 0;

        // Calculate the derivative
        for(size_t i = EDGE_CLIP_COUNT; i < (buffer_size - EDGE_CLIP_COUNT - 1); i++)
        {
            T deriv = (buffer[i + 1] & 0xFFF) - (buffer[i] & 0xFFF);
            derivative_buffer[i] = deriv;
            if(abs(deriv) > max_derivative)
                max_derivative = abs(deriv);
        }

        // Push all of the edges that exceed the threshold
        for(size_t i = EDGE_CLIP_COUNT; i < (buffer_size - EDGE_CLIP_COUNT - 1); i++)
        {
            T deriv = derivative_buffer[i];

            T threshold = max_derivative * THRESHOLD;

            if(abs(deriv) > threshold)
            {
                edges_buffer[edges_buffer_top++] = LineEdge_t(i, deriv);

                if(edges_buffer_top >= MAX_EDGES)
                    break;
            }
        }

        T left_edge, right_edge;
        left_edge = right_edge = buffer_size/2;


        // Rank edges based upon proximity
        for(size_t i = 0; i < edges_buffer_top; i++)
        {
            LineEdge_t* e = &(edges_buffer[i]);

            if(e->delta > 0)
            {
                left_edge = e->pos;
            }
            else
            if((e->pos < left_edge + MAX_LINE_WIDTH) &&
               (e->pos > left_edge + MIN_LINE_WIDTH))
            {
                right_edge = e->pos;
            }

        }

        m_centerpos = static_cast<float>((left_edge + right_edge)/2)/buffer_size;
    }

    static float getPos()
    {
        return m_centerpos;
    }

};

template<typename T, size_t buffer_size>
float LinecamProcessor<T, buffer_size>::m_centerpos = 0.0f;

template<typename T, size_t buffer_size>
T LinecamProcessor<T, buffer_size>::derivative_buffer[buffer_size - 1] = {0};

}

#endif // CAMERA_UTILS_H
