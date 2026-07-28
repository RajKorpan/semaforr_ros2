#include <semaforr/core/FORRAction.h>

int main()
{
  const FORRAction action(FORWARD, 1);
  return action.type == FORWARD ? 0 : 1;
}
