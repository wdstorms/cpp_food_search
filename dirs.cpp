#include "dirs.hpp"

const boost::unordered::unordered_map<std::pair<int, int>, std::string> dirs = {{{0, 1}, "East"}, {{0, -1}, "West"}, {{-1, 0}, "North"}, {{1, 0}, "South"}};
const boost::unordered::unordered_map<std::string, std::pair<int, int>> undirs = {{"East", {0, 1}}, {"West", {0, -1}}, {"North", {-1, 0}}, {"South", {1, 0}}};