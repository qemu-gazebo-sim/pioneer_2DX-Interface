@echo off

for /R src %%f in (*.cpp *.h *.hpp) do clang-format -i "%%f"
for /R include %%f in (*.cpp *.h *.hpp) do clang-format -i "%%f"
for /R test %%f in (*.cpp *.h *.hpp) do clang-format -i "%%f"

echo Done!