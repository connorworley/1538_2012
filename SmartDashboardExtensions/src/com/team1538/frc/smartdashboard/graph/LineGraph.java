package com.team1538.frc.smartdashboard.graph;

import java.awt.*;

import edu.wpi.first.smartdashboard.gui.Widget;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.smartdashboard.types.DataType;

import com.sun.tools.visualvm.charts.*;

@SuppressWarnings("serial")
public class LineGraph extends Widget {
	
	public static final DataType[] TYPES = { DataType.STRING };
	
	private static final int width = 800;
	private static final int height = 400;
	
	private static final int lines = 6;
	
	private SimpleXYChartSupport support;
	
	public void setValue(Object arg0) {
		
		String[] items = ((String)arg0).split(",");
		long[] values = new long[lines];
		for(int i = 0; i < values.length; i++) {
			values[i] = Long.parseLong(items[i]);
		}
		support.addValues(System.currentTimeMillis(), values);
		repaint();
	}

	public void init() {
		setPreferredSize(new Dimension(width, height));
		
		SimpleXYChartDescriptor descriptor = SimpleXYChartDescriptor.decimal(0, false, 200);
		
		for(int i = 0; i < lines; i++) {
			descriptor.addLineItems("Item "+i);
		}
		
		descriptor.setXAxisDescription("<html><div style=\"width:"+width+"px;\"></div></html>");
		descriptor.setYAxisDescription("<html><div style=\"width:"+height+"px;\"></div></html>");
		
		support = ChartFactory.createSimpleXYChart(descriptor);
		
		add(support.getChart(), BorderLayout.CENTER);
	}

	public void propertyChanged(Property arg0) {
	}
}