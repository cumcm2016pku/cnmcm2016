function road = RoadInstance(type, width, length, nCarriageway, v, vmin, waitTime, nCross, startPoint, endPoint)
road = struct('type', type, ...  % ��·���ͣ� 1ΪС���ڵ�·��2ΪС���ܱߵ�·
            'width', width, ...  % ƽ��ÿ�������Ŀ�ȣ���λΪ��
            'length', length, ... % ��·�ܳ��ȣ���λΪ��
            'nCarriageway', nCarriageway, ...  % �÷���ĳ�����Ŀ
            'v', v, ...   % ������ʻ�ٶȣ���λΪ��ÿ��
            'vmin', vmin, ...   % ͨ�����źŵƵĽ����ʱ�����ٵ�������ٶ�
            'waitTime', waitTime, ...  % �ڽ���·�ڵĵȴ�ʱ�䣬��λΪ��
            'nCross', nCross, ... % ����ڸ���
            'Start', startPoint, ....
            'End', endPoint, ...              %% ��������һЩĬ�ϲ���
            'alphaNumbers', [1, 0.85,  0.715, 0.575, 0.46], ... % �ۼ�ϵ�������֧��ͬһ������5����·
            'capacity', 0);
