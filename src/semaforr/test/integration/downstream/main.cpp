#include <semaforr/domain/action.hpp>

int main() {
  const semaforr::domain::Action action(semaforr::domain::ActionType::Forward,
                                        1U);
  return action.type() == semaforr::domain::ActionType::Forward ? 0 : 1;
}
