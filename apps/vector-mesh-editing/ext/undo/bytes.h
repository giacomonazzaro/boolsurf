#include <string>
#include <vector>

using byte = unsigned char;

inline std::vector<byte> make_diff(const byte* a, size_t a_size, const byte* b,
                                   size_t b_size, size_t window_size) {
    auto append = [](std::vector<byte>& a, const std::vector<byte>& b) {
        a.insert(a.end(), b.begin(), b.end());
    };

    auto result = std::vector<byte>{};
    result.reserve(std::max(a_size, b_size));
    auto window        = std::vector<byte>(window_size, 0);
    auto zero_run_size = (size_t)0;
    auto nonzero_run   = std::vector<byte>{};
    auto last_was_zero = true;

    auto append_nonzero_sequence = [&]() {
        auto nonzero_run_size = nonzero_run.size();
        auto hack             = (byte*)&nonzero_run_size;
        for (int i = 0; i < sizeof(size_t); i++) result.push_back(hack[i]);
        // printf("add nonzero_size: %ld\n", nonzero_run_size);
        // for (int i = 0; i < nonzero_run.size(); i++)
        //     printf("add byte: %c\n", nonzero_run[i]);

        append(result, nonzero_run);
    };

    auto append_zero_sequence = [&]() {
        auto hack = (byte*)&zero_run_size;
        for (int i = 0; i < sizeof(size_t); i++) result.push_back(hack[i]);
        // printf("add zero size: %ld\n", zero_run_size);
    };

    for (int i = 0;;) {
        auto diff_is_zero = true;
        for (int k = 0; k < window_size; k++) {
            auto aa = i < a_size ? a[i + k] : 0;
            auto bb = i < b_size ? b[i + k] : 0;
            auto x  = aa ^ bb;
            if (x) diff_is_zero = false;
            window[k] = x;
        }

        if (diff_is_zero) {
            if (last_was_zero) {
                zero_run_size += window_size;
            }
            if (!last_was_zero) {
                append_nonzero_sequence();
                nonzero_run.clear();
                zero_run_size = window_size;
            }
        }
        if (!diff_is_zero) {
            if (last_was_zero) {
                append_zero_sequence();
            }
            append(nonzero_run, window);
        }
        last_was_zero = diff_is_zero;
        i += window_size;

        if (i >= a_size && i >= b_size) {
            if (nonzero_run.size()) append_nonzero_sequence();
            if (zero_run_size) append_zero_sequence();
            break;
        }
    }
    // printf("buffer_size: %ld, diff_size: %ld\n", a_size, result.size());
    result.shrink_to_fit();
    return result;
}

template <typename T>
inline T parse_bytes(byte*& p) {
    auto result = *(T*)p;
    p += sizeof(T);
    return result;
}

inline void apply_diff(void* buffer, const std::vector<byte>& diff) {
    auto p = (byte*)buffer;
    auto d = (byte*)diff.data();
    while (true) {
        auto zero_size = parse_bytes<size_t>(d);
        if (d >= &diff.back()) return;
        p += zero_size;
        auto nonzero_size = parse_bytes<size_t>(d);
        for (int k = 0; k < nonzero_size; k++) {
            *p ^= *d;
            p += 1;
            d += 1;
            if (d >= &diff.back()) return;
        }
    }
}
