function realtimeparameters(fname)
    f = figure;

    ymax = -inf;
    ymin = inf;
    while true
        d =importdata(fname, ' ', 4);

        subplot(1, 2, 1);
        plot(d.data(:, 1), d.data(:, 3));

        subplot(1, 2, 2);
        cla;
        hold on;
        [nr, nc] = size(d.data);
        ymax = max([d.data(end, 4:nc) ymax]);
        ymin = min([d.data(end, 4:nc) ymin]);
        plot(4:nc, d.data(end, 4:nc));
        plot(4:nc, 0 * (4:nc));
        axis([-inf inf ymin ymax]);

        pause(0.5);
    end
end
