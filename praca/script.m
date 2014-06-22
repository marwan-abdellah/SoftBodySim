load mem_perf.mat

plot(A(:,1), A(:,2), 'h-', 'linewidth', 3)
x = xlabel('Przesunięcie (offset)')
y = ylabel('Przepustowość GB/s')
set(x, 'fontsize', 18)
set(y, 'fontsize', 18)
