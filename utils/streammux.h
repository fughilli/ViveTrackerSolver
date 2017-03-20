#ifndef STREAMMUX_H
#define STREAMMUX_H

namespace StreamMux
{
    /**
     * @brief Provides a single writer interface for a multiplexed stream.
     *
     * @tparam UWStreamAdapter The underlying write stream adapter that this
     *         stream is multiplexed over.
     * @tparam streamindex The index of this stream.
     */
    template<class UWStreamAdapter, uint8_t streamindex>
    struct MuxStreamWriter
    {
        static constexpr uint8_t index = streamindex;
        static void begin()
        {
            UWStreamAdapter::begin();
            UWStreamAdapter::write(index);
        }

        static void end()
        {
            UWStreamAdapter::end();
        }

        static void write(uint8_t data)
        {
            UWStreamAdapter::write(data);
        }
    };

    /**
     * @brief Provides a single writer interface for a multiplexed stream.
     *
     * @tparam UWStreamAdapter The underlying write stream adapter that this
     *         stream is multiplexed over.
     * @tparam streamindex The index of this stream.
     */
    template<class UWStreamAdapter, uint8_t streamindex>
    struct BoundedMuxStreamWriter : MuxStreamWriter<UWStreamAdapter, streamindex>
    {
        static uint32_t rem;
        static void begin(uint32_t len)
        {
            MuxStreamWriter<UWStreamAdapter, streamindex>::begin();
            UWStreamAdapter::write(rem >> 24);
            UWStreamAdapter::write((rem >> 16) & 0xFF);
            UWStreamAdapter::write((rem >> 8) & 0xFF);
            UWStreamAdapter::write(rem & 0xFF);
        }
    };

    template<class UWStreamAdapter, uint8_t streamindex>
    uint32_t BoundedMuxStreamWriter<UWStreamAdapter, streamindex>::rem = 0;

    template<uint8_t streamindex>
    struct MuxStreamReader
    {
        static constexpr uint8_t index = streamindex;
        static void rxbegincallback();
        static void rxendcallback();
        static void rxcallback(uint8_t data);
    };

    template<class TLStreamAdapter, uint8_t streamindex>
    struct MuxStreamReaderAdapter : MuxStreamReader<streamindex>
    {
        static void rxbegincallback()
        {
            TLStreamAdapter::rxbegincallback();
        }

        static void rxendcallback()
        {
            TLStreamAdapter::rxendcallback();
        }

        static void rxcallback(uint8_t data)
        {
            TLStreamAdapter::rxcallback(data);
        }
    };

    template<class MStreamReader>
    void rxbegincallbackdelegator(uint8_t index)
    {
        if(MStreamReader::index == index)
            MStreamReader::rxbegincallback();
    }

    template<class MStreamReader1, class MStreamReader2, class ... MStreamReaders>
    void rxbegincallbackdelegator(uint8_t index)
    {
        rxbegincallbackdelegator<MStreamReader1>(index);
        rxbegincallbackdelegator<MStreamReader2, MStreamReaders...>(index);
    }

    template<class MStreamReader>
    void rxendcallbackdelegator(uint8_t index)
    {
        if(MStreamReader::index == index)
            MStreamReader::rxendcallback();
    }

    template<class MStreamReader1, class MStreamReader2, class ... MStreamReaders>
    void rxendcallbackdelegator(uint8_t index)
    {
        rxendcallbackdelegator<MStreamReader1>(index);
        rxendcallbackdelegator<MStreamReader2, MStreamReaders...>(index);
    }

    template<class MStreamReader>
    void rxcallbackdelegator(uint8_t index, uint8_t data)
    {
        if(MStreamReader::index == index)
            MStreamReader::rxcallback(data);
    }

    template<class MStreamReader1, class MStreamReader2, class ... MStreamReaders>
    void rxcallbackdelegator(uint8_t index, uint8_t data)
    {
        rxcallbackdelegator<MStreamReader1>(index, data);
        rxcallbackdelegator<MStreamReader2, MStreamReaders...>(index, data);
    }

    /**
     * @brief Provides for multiple reader interfaces on a single multiplexed
     *        stream.
     *
     * @tparam MStreamReaders The stream readers that are on this stream.
     */
    template<class ... MStreamReaders>
    struct StreamReaderMultiplexer
    {
        /**
         * @brief Represents the state of the stream reader multiplexer.
         */
        enum class StreamMuxState_t : uint8_t
        {
            STATE_IDLE = 0,
            STATE_BEGUN_NO_INDEX,
            STATE_BEGUN
        };

        static StreamMuxState_t state;
        static uint8_t index;

        static void rxbegincallback()
        {
            /*
             * Set our state. We can't do any other work here, because we don't
             * yet know the stream index.
             */
            state = StreamMuxState_t::STATE_BEGUN_NO_INDEX;
        }

        static void rxendcallback()
        {
            // Set our state.
            state = StreamMuxState_t::STATE_IDLE;

            // Figure out which of MStreamReaders to call rxendcallback() on
            rxendcallbackdelegator<MStreamReaders...>(index);
        }

        static void rxcallback(uint8_t data)
        {
            switch(state)
            {
                case StreamMuxState_t::STATE_BEGUN_NO_INDEX:
                    /*
                     * We just got the stream index; set the state, set the
                     * index, and then call the appropriate rxbegincallback.
                     */
                    state = StreamMuxState_t::STATE_BEGUN;
                    index = data;

                    rxbegincallbackdelegator<MStreamReaders...>(index);

                    break;
                case StreamMuxState_t::STATE_BEGUN:
                    /*
                     * We already have the stream index; just pass the byte
                     * along to the appropriate rxcallback.
                     */
                    rxcallbackdelegator<MStreamReaders...>(index, data);

                    break;

                case StreamMuxState_t::STATE_IDLE:
                default:
                    /*
                     * We shouldn't end up here...
                     */
                    break;
            }
        }
    };

    // Initialize static members.
    template<class ... MStreamReaders>
    typename StreamReaderMultiplexer<MStreamReaders...>::StreamMuxState_t
        StreamReaderMultiplexer<MStreamReaders...>::state =
        StreamReaderMultiplexer<MStreamReaders...>::StreamMuxState_t::STATE_IDLE;
    template<class ... MStreamReaders>
    uint8_t StreamReaderMultiplexer<MStreamReaders...>::index = 0;
}

#endif //STREAMMUX_H
