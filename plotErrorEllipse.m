function plotErrorEllipse(x, y, Pmatrix, varargin)
    % オプションのパーサーを作成
    p = inputParser;
    
    % デフォルト値の設定
    defaultConfidence = 0.95;  % 95%信頼区間
    defaultPoints = 100;       % 楕円の描画ポイント数
    
    % オプションの追加
    addOptional(p, 'confidence', defaultConfidence, @(x) x > 0 && x < 1);
    addOptional(p, 'points', defaultPoints, @(x) x > 0 && isfloat(x));
    
    % 入力値の解析
    parse(p, varargin{:});
    
    % カイ二乗値の計算（信頼区間に基づく）
    chi2_val = chi2inv(p.Results.confidence, 2);
    
    % 楕円の描画ポイント数
    npoints = p.Results.points;
    
    % 楕円の角度
    theta = linspace(0, 2*pi, npoints);
    
    % 2x2の共分散行列から固有値と固有ベクトルを計算
    [eigvec, eigval] = eig(Pmatrix);
    
    % 楕円の半径を計算（信頼区間を考慮）
    rx = sqrt(chi2_val * eigval(1,1));
    ry = sqrt(chi2_val * eigval(2,2));
    
    % 回転角度を計算
    angle = atan2(eigvec(2,1), eigvec(1,1));
    
    % 基本楕円の座標を生成
    ellipse_x = rx * cos(theta);
    ellipse_y = ry * sin(theta);
    
    % 回転行列
    R = [cos(angle) -sin(angle);
         sin(angle)  cos(angle)];
    
    % 楕円の座標を回転
    points = R * [ellipse_x; ellipse_y];
    
    % 楕円を平行移動
    points(1,:) = points(1,:) + x;
    points(2,:) = points(2,:) + y;
    
    % 楕円を描画（指定された色で）
    plot(points(1,:), points(2,:), 'Color', '#66cdaa', 'LineWidth', 0.5);
    hold on;
    plot(x, y,'.','Color','#66cdaa','MarkerSize', 12);
end
