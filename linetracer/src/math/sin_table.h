#pragma once
#include <avr/pgmspace.h>
#include <stdint.h>

// 0~90도, 1도 간격 sin 테이블. 값은 Q15 (32767 = 1.0)
namespace trig
{

constexpr uint8_t SIN_TABLE_SIZE = 91;
constexpr int16_t Q15_ONE = 32767;
constexpr double DEG_TO_RAD = 3.14159265358979 / 180.0;

// C++11 constexpr 는 return 하나만 허용하므로 테일러 급수를 재귀로 푼다.
// avr-gcc 의 double 은 32비트라 항을 더 늘려도 정밀도는 그대로다.
constexpr double next_term(double term, double x2, uint8_t k)
{
    return -term * x2 / ((2 * k + 2) * (2 * k + 3));
}

constexpr double sin_series(double x2, double term, uint8_t k)
{
    return k > 7 ? 0.0 : term + sin_series(x2, next_term(term, x2, k), k + 1);
}

constexpr double sin_rad(double x) { return sin_series(x * x, x, 0); }

constexpr int16_t sin_q15_at(uint8_t deg)
{
    return static_cast<int16_t>(sin_rad(deg * DEG_TO_RAD) * Q15_ONE + 0.5);
}

// std::index_sequence 는 C++14 라서 직접 만든다
template <uint8_t... I> struct Indices
{
};

template <uint8_t N, uint8_t... I>
struct MakeIndices : MakeIndices<N - 1, N - 1, I...>
{
};

template <uint8_t... I> struct MakeIndices<0, I...>
{
    using Type = Indices<I...>;
};

template <typename T> struct SinTable;

// 템플릿 정적 멤버라 헤더에 정의해도 여러 번역 단위에서 하나로 합쳐진다
template <uint8_t... I> struct SinTable<Indices<I...>>
{
    static const int16_t VALUES[sizeof...(I)];
};

template <uint8_t... I>
const int16_t SinTable<Indices<I...>>::VALUES[sizeof...(I)] PROGMEM = {
    sin_q15_at(I)...};

using SinTableQ15 = SinTable<MakeIndices<SIN_TABLE_SIZE>::Type>;

static_assert(sin_q15_at(0) == 0, "sin(0) must be 0");
static_assert(sin_q15_at(90) == Q15_ONE, "sin(90) must be 1.0");

// PROGMEM 이라 배열을 직접 인덱싱하면 RAM 주소로 읽어 버린다
inline int16_t sin_q15(uint8_t deg)
{
    return static_cast<int16_t>(pgm_read_word(&SinTableQ15::VALUES[deg]));
}

} // namespace trig
