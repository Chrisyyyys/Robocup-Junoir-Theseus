#ifndef BITSET_H
#define BITSET_H

template <unsigned size>
class Bitset
{
private:
    static constexpr unsigned W = 8; // bits per uint8_t
    static constexpr unsigned WORDS = (size + W - 1) / W;

    unsigned char mem[WORDS]{}; // zero-initialized

public:
    bool get(unsigned i) const
    {
        if (i >= size) return false;              // or throw/assert
        const unsigned word = i / W;
        const unsigned bit  = i % W;
        return (mem[word] >> bit) & 0x1u;
    }

    void set(unsigned i, bool b)
    {
        if (i >= size) return;                    // or throw/assert
        const unsigned word = i / W;
        const unsigned bit  = i % W;
        const unsigned char mask = static_cast<unsigned char>(1u << bit);

        if (b) mem[word] |= mask;
        else   mem[word] &= static_cast<unsigned char>(~mask);
    }
};

#endif