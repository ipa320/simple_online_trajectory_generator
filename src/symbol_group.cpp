#include "sotg/symbol_group.hpp"

#include <algorithm>
#include <string.h>

using namespace SOTG;

bool SymbolGroup::operator==(const SymbolGroup& symbol_group) const
{
    return symbol_group.getSymbols() == symbols_;
}
bool SymbolGroup::operator!=(const SymbolGroup& symbol_group) const
{
    return symbol_group.getSymbols() != symbols_;
}

std::string SymbolGroup::str() const
{
    std::string output;
    output.append("[ ");
    for (auto& symbol : symbols_) {
        output.append(symbol);
        output.append(",");
    }
    output.pop_back();
    output.append(" ]");

    return output;
}

size_t SymbolGroup::getIndex(std::string key)
{
    auto it = std::find(symbols_.begin(), symbols_.end(), key);
    if (it != symbols_.end()) {
        return it - symbols_.begin();
    } else {
        throw std::runtime_error("Symbol \"" + key + "\" not in symbol group");
    }
}
