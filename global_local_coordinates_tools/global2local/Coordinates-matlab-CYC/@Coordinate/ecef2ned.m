function ned=ecef2ned(obj,ecef,blh0)

%列向量表示
blh=obj.ecef2blh(ecef);
ned=obj.blh2ned(blh0,blh);

end