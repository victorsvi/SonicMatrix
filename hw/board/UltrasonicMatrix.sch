<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.4.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="20" unitdist="mil" unit="mil" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="5" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="TC4427EOA">
<packages>
<package name="SOIC127P599X175-8N">
<circle x="-4.25" y="2.505" radius="0.1" width="0.2" layer="21"/>
<circle x="-4.25" y="2.505" radius="0.1" width="0.2" layer="51"/>
<wire x1="-1.995" y1="2.5" x2="1.995" y2="2.5" width="0.127" layer="51"/>
<wire x1="-1.995" y1="-2.5" x2="1.995" y2="-2.5" width="0.127" layer="51"/>
<wire x1="-1.995" y1="2.525" x2="1.995" y2="2.525" width="0.127" layer="21"/>
<wire x1="-1.995" y1="-2.525" x2="1.995" y2="-2.525" width="0.127" layer="21"/>
<wire x1="-1.995" y1="2.5" x2="-1.995" y2="-2.5" width="0.127" layer="51"/>
<wire x1="1.995" y1="2.5" x2="1.995" y2="-2.5" width="0.127" layer="51"/>
<wire x1="-3.71" y1="2.75" x2="3.71" y2="2.75" width="0.05" layer="39"/>
<wire x1="-3.71" y1="-2.75" x2="3.71" y2="-2.75" width="0.05" layer="39"/>
<wire x1="-3.71" y1="2.75" x2="-3.71" y2="-2.75" width="0.05" layer="39"/>
<wire x1="3.71" y1="2.75" x2="3.71" y2="-2.75" width="0.05" layer="39"/>
<text x="-3.575" y="-2.702" size="1.27" layer="27" align="top-left">&gt;VALUE</text>
<text x="-3.575" y="2.702" size="1.27" layer="25">&gt;NAME</text>
<smd name="1" x="-2.924" y="1.905" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="2" x="-2.924" y="0.635" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="3" x="-2.924" y="-0.635" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="4" x="-2.924" y="-1.905" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="5" x="2.924" y="-1.905" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="6" x="2.924" y="-0.635" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="7" x="2.924" y="0.635" dx="2.088" dy="0.6" layer="1" roundness="25"/>
<smd name="8" x="2.924" y="1.905" dx="2.088" dy="0.6" layer="1" roundness="25"/>
</package>
</packages>
<symbols>
<symbol name="TC4427EOA">
<wire x1="-12.7" y1="12.7" x2="12.7" y2="12.7" width="0.41" layer="94"/>
<wire x1="12.7" y1="12.7" x2="12.7" y2="-12.7" width="0.41" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="-12.7" y2="-12.7" width="0.41" layer="94"/>
<wire x1="-12.7" y1="-12.7" x2="-12.7" y2="12.7" width="0.41" layer="94"/>
<text x="-12.7" y="13.7" size="2.0828" layer="95" ratio="10" rot="SR0">&gt;NAME</text>
<text x="-12.7" y="-16.7" size="2.0828" layer="96" ratio="10" rot="SR0">&gt;VALUE</text>
<pin name="IN_A" x="-17.78" y="5.08" length="middle" direction="in"/>
<pin name="IN_B" x="-17.78" y="2.54" length="middle" direction="in"/>
<pin name="NC" x="-17.78" y="-2.54" length="middle"/>
<pin name="VDD" x="17.78" y="10.16" length="middle" direction="pwr" rot="R180"/>
<pin name="!OUT_A" x="17.78" y="5.08" length="middle" direction="out" rot="R180"/>
<pin name="!OUT_B" x="17.78" y="2.54" length="middle" direction="out" rot="R180"/>
<pin name="GND" x="17.78" y="-7.62" length="middle" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="TC4427EOA" prefix="U">
<gates>
<gate name="A" symbol="TC4427EOA" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOIC127P599X175-8N">
<connects>
<connect gate="A" pin="!OUT_A" pad="7"/>
<connect gate="A" pin="!OUT_B" pad="5"/>
<connect gate="A" pin="GND" pad="3"/>
<connect gate="A" pin="IN_A" pad="2"/>
<connect gate="A" pin="IN_B" pad="4"/>
<connect gate="A" pin="NC" pad="1 8"/>
<connect gate="A" pin="VDD" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="AVAILABILITY" value="Unavailable"/>
<attribute name="DESCRIPTION" value="  Low-Side Gate Driver IC Non-Inverting 8-SOIC "/>
<attribute name="MF" value="Microchip"/>
<attribute name="MP" value="TC4427EOA"/>
<attribute name="PACKAGE" value="SOIC-8 Microchip"/>
<attribute name="PRICE" value="None"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="rcl" urn="urn:adsk.eagle:library:334">
<description>&lt;b&gt;Resistors, Capacitors, Inductors&lt;/b&gt;&lt;p&gt;
Based on the previous libraries:
&lt;ul&gt;
&lt;li&gt;r.lbr
&lt;li&gt;cap.lbr 
&lt;li&gt;cap-fe.lbr
&lt;li&gt;captant.lbr
&lt;li&gt;polcap.lbr
&lt;li&gt;ipc-smd.lbr
&lt;/ul&gt;
All SMD packages are defined according to the IPC specifications and  CECC&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;p&gt;
&lt;p&gt;
for Electrolyt Capacitors see also :&lt;p&gt;
www.bccomponents.com &lt;p&gt;
www.panasonic.com&lt;p&gt;
www.kemet.com&lt;p&gt;
http://www.secc.co.jp/pdf/os_e/2004/e_os_all.pdf &lt;b&gt;(SANYO)&lt;/b&gt;
&lt;p&gt;
for trimmer refence see : &lt;u&gt;www.electrospec-inc.com/cross_references/trimpotcrossref.asp&lt;/u&gt;&lt;p&gt;

&lt;table border=0 cellspacing=0 cellpadding=0 width="100%" cellpaddding=0&gt;
&lt;tr valign="top"&gt;

&lt;! &lt;td width="10"&gt;&amp;nbsp;&lt;/td&gt;
&lt;td width="90%"&gt;

&lt;b&gt;&lt;font color="#0000FF" size="4"&gt;TRIM-POT CROSS REFERENCE&lt;/font&gt;&lt;/b&gt;
&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=2&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;RECTANGULAR MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BOURNS&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BI&amp;nbsp;TECH&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;DALE-VISHAY&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PHILIPS/MEPCO&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MURATA&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PANASONIC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;SPECTROL&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MILSPEC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;&lt;TD&gt;&amp;nbsp;&lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3 &gt;
      3005P&lt;BR&gt;
      3006P&lt;BR&gt;
      3006W&lt;BR&gt;
      3006Y&lt;BR&gt;
      3009P&lt;BR&gt;
      3009W&lt;BR&gt;
      3009Y&lt;BR&gt;
      3057J&lt;BR&gt;
      3057L&lt;BR&gt;
      3057P&lt;BR&gt;
      3057Y&lt;BR&gt;
      3059J&lt;BR&gt;
      3059L&lt;BR&gt;
      3059P&lt;BR&gt;
      3059Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      89P&lt;BR&gt;
      89W&lt;BR&gt;
      89X&lt;BR&gt;
      89PH&lt;BR&gt;
      76P&lt;BR&gt;
      89XH&lt;BR&gt;
      78SLT&lt;BR&gt;
      78L&amp;nbsp;ALT&lt;BR&gt;
      56P&amp;nbsp;ALT&lt;BR&gt;
      78P&amp;nbsp;ALT&lt;BR&gt;
      T8S&lt;BR&gt;
      78L&lt;BR&gt;
      56P&lt;BR&gt;
      78P&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      T18/784&lt;BR&gt;
      783&lt;BR&gt;
      781&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2199&lt;BR&gt;
      1697/1897&lt;BR&gt;
      1680/1880&lt;BR&gt;
      2187&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      8035EKP/CT20/RJ-20P&lt;BR&gt;
      -&lt;BR&gt;
      RJ-20X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      1211L&lt;BR&gt;
      8012EKQ&amp;nbsp;ALT&lt;BR&gt;
      8012EKR&amp;nbsp;ALT&lt;BR&gt;
      1211P&lt;BR&gt;
      8012EKJ&lt;BR&gt;
      8012EKL&lt;BR&gt;
      8012EKQ&lt;BR&gt;
      8012EKR&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      2101P&lt;BR&gt;
      2101W&lt;BR&gt;
      2101Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2102L&lt;BR&gt;
      2102S&lt;BR&gt;
      2102Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVMCOG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      43P&lt;BR&gt;
      43W&lt;BR&gt;
      43Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      40L&lt;BR&gt;
      40P&lt;BR&gt;
      40Y&lt;BR&gt;
      70Y-T602&lt;BR&gt;
      70L&lt;BR&gt;
      70P&lt;BR&gt;
      70Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SQUARE MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
   &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3250L&lt;BR&gt;
      3250P&lt;BR&gt;
      3250W&lt;BR&gt;
      3250X&lt;BR&gt;
      3252P&lt;BR&gt;
      3252W&lt;BR&gt;
      3252X&lt;BR&gt;
      3260P&lt;BR&gt;
      3260W&lt;BR&gt;
      3260X&lt;BR&gt;
      3262P&lt;BR&gt;
      3262W&lt;BR&gt;
      3262X&lt;BR&gt;
      3266P&lt;BR&gt;
      3266W&lt;BR&gt;
      3266X&lt;BR&gt;
      3290H&lt;BR&gt;
      3290P&lt;BR&gt;
      3290W&lt;BR&gt;
      3292P&lt;BR&gt;
      3292W&lt;BR&gt;
      3292X&lt;BR&gt;
      3296P&lt;BR&gt;
      3296W&lt;BR&gt;
      3296X&lt;BR&gt;
      3296Y&lt;BR&gt;
      3296Z&lt;BR&gt;
      3299P&lt;BR&gt;
      3299W&lt;BR&gt;
      3299X&lt;BR&gt;
      3299Y&lt;BR&gt;
      3299Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64P&amp;nbsp;ALT&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      64X&amp;nbsp;ALT&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66P&lt;BR&gt;
      66W&lt;BR&gt;
      66X&lt;BR&gt;
      67P&lt;BR&gt;
      67W&lt;BR&gt;
      67X&lt;BR&gt;
      67Y&lt;BR&gt;
      67Z&lt;BR&gt;
      68P&lt;BR&gt;
      68W&lt;BR&gt;
      68X&lt;BR&gt;
      67Y&amp;nbsp;ALT&lt;BR&gt;
      67Z&amp;nbsp;ALT&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      5050&lt;BR&gt;
      5091&lt;BR&gt;
      5080&lt;BR&gt;
      5087&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T63YB&lt;BR&gt;
      T63XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      5887&lt;BR&gt;
      5891&lt;BR&gt;
      5880&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T93Z&lt;BR&gt;
      T93YA&lt;BR&gt;
      T93XA&lt;BR&gt;
      T93YB&lt;BR&gt;
      T93XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKW&lt;BR&gt;
      8026EKM&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKB&lt;BR&gt;
      8026EKM&lt;BR&gt;
      1309X&lt;BR&gt;
      1309P&lt;BR&gt;
      1309W&lt;BR&gt;
      8024EKP&lt;BR&gt;
      8024EKW&lt;BR&gt;
      8024EKN&lt;BR&gt;
      RJ-9P/CT9P&lt;BR&gt;
      RJ-9W&lt;BR&gt;
      RJ-9X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3105P/3106P&lt;BR&gt;
      3105W/3106W&lt;BR&gt;
      3105X/3106X&lt;BR&gt;
      3105Y/3106Y&lt;BR&gt;
      3105Z/3105Z&lt;BR&gt;
      3102P&lt;BR&gt;
      3102W&lt;BR&gt;
      3102X&lt;BR&gt;
      3102Y&lt;BR&gt;
      3102Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMCBG&lt;BR&gt;
      EVMCCG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      55-1-X&lt;BR&gt;
      55-4-X&lt;BR&gt;
      55-3-X&lt;BR&gt;
      55-2-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      50-2-X&lt;BR&gt;
      50-4-X&lt;BR&gt;
      50-3-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      64Y&lt;BR&gt;
      64Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3323P&lt;BR&gt;
      3323S&lt;BR&gt;
      3323W&lt;BR&gt;
      3329H&lt;BR&gt;
      3329P&lt;BR&gt;
      3329W&lt;BR&gt;
      3339H&lt;BR&gt;
      3339P&lt;BR&gt;
      3339W&lt;BR&gt;
      3352E&lt;BR&gt;
      3352H&lt;BR&gt;
      3352K&lt;BR&gt;
      3352P&lt;BR&gt;
      3352T&lt;BR&gt;
      3352V&lt;BR&gt;
      3352W&lt;BR&gt;
      3362H&lt;BR&gt;
      3362M&lt;BR&gt;
      3362P&lt;BR&gt;
      3362R&lt;BR&gt;
      3362S&lt;BR&gt;
      3362U&lt;BR&gt;
      3362W&lt;BR&gt;
      3362X&lt;BR&gt;
      3386B&lt;BR&gt;
      3386C&lt;BR&gt;
      3386F&lt;BR&gt;
      3386H&lt;BR&gt;
      3386K&lt;BR&gt;
      3386M&lt;BR&gt;
      3386P&lt;BR&gt;
      3386S&lt;BR&gt;
      3386W&lt;BR&gt;
      3386X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      25P&lt;BR&gt;
      25S&lt;BR&gt;
      25RX&lt;BR&gt;
      82P&lt;BR&gt;
      82M&lt;BR&gt;
      82PA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      91E&lt;BR&gt;
      91X&lt;BR&gt;
      91T&lt;BR&gt;
      91B&lt;BR&gt;
      91A&lt;BR&gt;
      91V&lt;BR&gt;
      91W&lt;BR&gt;
      25W&lt;BR&gt;
      25V&lt;BR&gt;
      25P&lt;BR&gt;
      -&lt;BR&gt;
      25S&lt;BR&gt;
      25U&lt;BR&gt;
      25RX&lt;BR&gt;
      25X&lt;BR&gt;
      72XW&lt;BR&gt;
      72XL&lt;BR&gt;
      72PM&lt;BR&gt;
      72RX&lt;BR&gt;
      -&lt;BR&gt;
      72PX&lt;BR&gt;
      72P&lt;BR&gt;
      72RXW&lt;BR&gt;
      72RXL&lt;BR&gt;
      72X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T7YB&lt;BR&gt;
      T7YA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      TXD&lt;BR&gt;
      TYA&lt;BR&gt;
      TYP&lt;BR&gt;
      -&lt;BR&gt;
      TYD&lt;BR&gt;
      TX&lt;BR&gt;
      -&lt;BR&gt;
      150SX&lt;BR&gt;
      100SX&lt;BR&gt;
      102T&lt;BR&gt;
      101S&lt;BR&gt;
      190T&lt;BR&gt;
      150TX&lt;BR&gt;
      101&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      101SX&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ET6P&lt;BR&gt;
      ET6S&lt;BR&gt;
      ET6X&lt;BR&gt;
      RJ-6W/8014EMW&lt;BR&gt;
      RJ-6P/8014EMP&lt;BR&gt;
      RJ-6X/8014EMX&lt;BR&gt;
      TM7W&lt;BR&gt;
      TM7P&lt;BR&gt;
      TM7X&lt;BR&gt;
      -&lt;BR&gt;
      8017SMS&lt;BR&gt;
      -&lt;BR&gt;
      8017SMB&lt;BR&gt;
      8017SMA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      CT-6W&lt;BR&gt;
      CT-6H&lt;BR&gt;
      CT-6P&lt;BR&gt;
      CT-6R&lt;BR&gt;
      -&lt;BR&gt;
      CT-6V&lt;BR&gt;
      CT-6X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKV&lt;BR&gt;
      -&lt;BR&gt;
      8038EKX&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKP&lt;BR&gt;
      8038EKZ&lt;BR&gt;
      8038EKW&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3321H&lt;BR&gt;
      3321P&lt;BR&gt;
      3321N&lt;BR&gt;
      1102H&lt;BR&gt;
      1102P&lt;BR&gt;
      1102T&lt;BR&gt;
      RVA0911V304A&lt;BR&gt;
      -&lt;BR&gt;
      RVA0911H413A&lt;BR&gt;
      RVG0707V100A&lt;BR&gt;
      RVA0607V(H)306A&lt;BR&gt;
      RVA1214H213A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3104B&lt;BR&gt;
      3104C&lt;BR&gt;
      3104F&lt;BR&gt;
      3104H&lt;BR&gt;
      -&lt;BR&gt;
      3104M&lt;BR&gt;
      3104P&lt;BR&gt;
      3104S&lt;BR&gt;
      3104W&lt;BR&gt;
      3104X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      EVMQ0G&lt;BR&gt;
      EVMQIG&lt;BR&gt;
      EVMQ3G&lt;BR&gt;
      EVMS0G&lt;BR&gt;
      EVMQ0G&lt;BR&gt;
      EVMG0G&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMK4GA00B&lt;BR&gt;
      EVM30GA00B&lt;BR&gt;
      EVMK0GA00B&lt;BR&gt;
      EVM38GA00B&lt;BR&gt;
      EVMB6&lt;BR&gt;
      EVLQ0&lt;BR&gt;
      -&lt;BR&gt;
      EVMMSG&lt;BR&gt;
      EVMMBG&lt;BR&gt;
      EVMMAG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMMCS&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM0&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM3&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      62-3-1&lt;BR&gt;
      62-1-2&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67R&lt;BR&gt;
      -&lt;BR&gt;
      67P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67X&lt;BR&gt;
      63V&lt;BR&gt;
      63S&lt;BR&gt;
      63M&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63H&lt;BR&gt;
      63P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;&amp;nbsp;&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=3&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT color="#0000FF" SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SMD TRIM-POT CROSS REFERENCE&lt;/B&gt;&lt;/FONT&gt;
      &lt;P&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3224G&lt;BR&gt;
      3224J&lt;BR&gt;
      3224W&lt;BR&gt;
      3269P&lt;BR&gt;
      3269W&lt;BR&gt;
      3269X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      44G&lt;BR&gt;
      44J&lt;BR&gt;
      44W&lt;BR&gt;
      84P&lt;BR&gt;
      84W&lt;BR&gt;
      84X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST63Z&lt;BR&gt;
      ST63Y&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST5P&lt;BR&gt;
      ST5W&lt;BR&gt;
      ST5X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3314G&lt;BR&gt;
      3314J&lt;BR&gt;
      3364A/B&lt;BR&gt;
      3364C/D&lt;BR&gt;
      3364W/X&lt;BR&gt;
      3313G&lt;BR&gt;
      3313J&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      23B&lt;BR&gt;
      23A&lt;BR&gt;
      21X&lt;BR&gt;
      21W&lt;BR&gt;
      -&lt;BR&gt;
      22B&lt;BR&gt;
      22A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST5YL/ST53YL&lt;BR&gt;
      ST5YJ/5T53YJ&lt;BR&gt;
      ST-23A&lt;BR&gt;
      ST-22B&lt;BR&gt;
      ST-22&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST-4B&lt;BR&gt;
      ST-4A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST-3B&lt;BR&gt;
      ST-3A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVM-6YS&lt;BR&gt;
      EVM-1E&lt;BR&gt;
      EVM-1G&lt;BR&gt;
      EVM-1D&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      G4B&lt;BR&gt;
      G4A&lt;BR&gt;
      TR04-3S1&lt;BR&gt;
      TRG04-2S1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      DVR-43A&lt;BR&gt;
      CVR-42C&lt;BR&gt;
      CVR-42A/C&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;
&lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;ALT =&amp;nbsp;ALTERNATE&lt;/B&gt;&lt;/FONT&gt;
&lt;P&gt;

&amp;nbsp;
&lt;P&gt;
&lt;/td&gt;
&lt;/tr&gt;
&lt;/table&gt;</description>
<packages>
<package name="C0402" urn="urn:adsk.eagle:footprint:23121/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C0504" urn="urn:adsk.eagle:footprint:23122/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.294" y1="0.559" x2="0.294" y2="0.559" width="0.1016" layer="51"/>
<wire x1="-0.294" y1="-0.559" x2="0.294" y2="-0.559" width="0.1016" layer="51"/>
<smd name="1" x="-0.7" y="0" dx="1" dy="1.3" layer="1"/>
<smd name="2" x="0.7" y="0" dx="1" dy="1.3" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.6604" y1="-0.6223" x2="-0.2804" y2="0.6276" layer="51"/>
<rectangle x1="0.2794" y1="-0.6223" x2="0.6594" y2="0.6276" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C0603" urn="urn:adsk.eagle:footprint:23123/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C0805" urn="urn:adsk.eagle:footprint:23124/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0922" y1="-0.7239" x2="-0.3421" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C1206" urn="urn:adsk.eagle:footprint:23125/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="C1210" urn="urn:adsk.eagle:footprint:23126/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-1.2954" x2="-0.9517" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3045" x2="1.7018" y2="1.2954" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="C1310" urn="urn:adsk.eagle:footprint:23127/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.294" y1="0.559" x2="0.294" y2="0.559" width="0.1016" layer="51"/>
<wire x1="-0.294" y1="-0.559" x2="0.294" y2="-0.559" width="0.1016" layer="51"/>
<smd name="1" x="-0.7" y="0" dx="1" dy="1.3" layer="1"/>
<smd name="2" x="0.7" y="0" dx="1" dy="1.3" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.6604" y1="-0.6223" x2="-0.2804" y2="0.6276" layer="51"/>
<rectangle x1="0.2794" y1="-0.6223" x2="0.6594" y2="0.6276" layer="51"/>
<rectangle x1="-0.1001" y1="-0.3" x2="0.1001" y2="0.3" layer="35"/>
</package>
<package name="C1608" urn="urn:adsk.eagle:footprint:23128/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C1812" urn="urn:adsk.eagle:footprint:23129/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="1.983" x2="2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-1.983" x2="-2.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-1.983" x2="-2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<wire x1="2.973" y1="1.983" x2="2.973" y2="-1.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-1.905" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.3" y1="-0.4001" x2="0.3" y2="0.4001" layer="35"/>
</package>
<package name="C1825" urn="urn:adsk.eagle:footprint:23130/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="3.483" x2="2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-3.483" x2="-2.973" y2="-3.483" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-3.483" x2="-2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<wire x1="2.973" y1="3.483" x2="2.973" y2="-3.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-1.905" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.7" y1="-0.7" x2="0.7" y2="0.7" layer="35"/>
</package>
<package name="C2012" urn="urn:adsk.eagle:footprint:23131/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0922" y1="-0.7239" x2="-0.3421" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C3216" urn="urn:adsk.eagle:footprint:23132/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.3" y1="-0.5001" x2="0.3" y2="0.5001" layer="35"/>
</package>
<package name="C3225" urn="urn:adsk.eagle:footprint:23133/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-1.2954" x2="-0.9517" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3045" x2="1.7018" y2="1.2954" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="C4532" urn="urn:adsk.eagle:footprint:23134/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="1.983" x2="2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-1.983" x2="-2.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-1.983" x2="-2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<wire x1="2.973" y1="1.983" x2="2.973" y2="-1.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-1.905" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="C4564" urn="urn:adsk.eagle:footprint:23135/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="3.483" x2="2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-3.483" x2="-2.973" y2="-3.483" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-3.483" x2="-2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<wire x1="2.973" y1="3.483" x2="2.973" y2="-3.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-1.905" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="C025-024X044" urn="urn:adsk.eagle:footprint:23136/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.778" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.778" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-025X050" urn="urn:adsk.eagle:footprint:23137/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.5 x 5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-030X050" urn="urn:adsk.eagle:footprint:23138/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 3 x 5 mm</description>
<wire x1="-2.159" y1="1.524" x2="2.159" y2="1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.524" x2="-2.159" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.27" x2="2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.27" x2="-2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.524" x2="2.413" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.27" x2="-2.159" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.524" x2="2.413" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.27" x2="-2.159" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-040X050" urn="urn:adsk.eagle:footprint:23139/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 4 x 5 mm</description>
<wire x1="-2.159" y1="1.905" x2="2.159" y2="1.905" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.905" x2="-2.159" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.651" x2="2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.651" x2="-2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.905" x2="2.413" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.651" x2="-2.159" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.905" x2="2.413" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.651" x2="-2.159" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-050X050" urn="urn:adsk.eagle:footprint:23140/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 5 x 5 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-060X050" urn="urn:adsk.eagle:footprint:23141/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 6 x 5 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-024X070" urn="urn:adsk.eagle:footprint:23142/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm + 5 mm, outline 2.4 x 7 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.191" y1="-1.143" x2="-3.9624" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-4.191" y1="1.143" x2="-3.9624" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-0.635" x2="-4.191" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="0.635" x2="-4.191" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.699" y1="-0.635" x2="-4.699" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="1.143" x2="-2.5654" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.143" x2="-2.5654" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.81" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-025X075" urn="urn:adsk.eagle:footprint:23143/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 2.5 x 7.5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="1.016" x2="4.953" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="4.699" y1="1.27" x2="4.953" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.27" x2="4.953" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="1.27" x2="4.699" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.27" x2="2.794" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="0.762" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-0.762" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.254" x2="2.413" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.159" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.159" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-035X075" urn="urn:adsk.eagle:footprint:23144/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 3.5 x 7.5 mm</description>
<wire x1="-2.159" y1="1.778" x2="2.159" y2="1.778" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.778" x2="-2.159" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.524" x2="-2.413" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.778" x2="2.413" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.524" x2="-2.159" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.778" x2="2.413" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.524" x2="-2.159" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="2.794" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.524" x2="2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.413" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.508" x2="2.413" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-045X075" urn="urn:adsk.eagle:footprint:23145/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 4.5 x 7.5 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="2.032" x2="4.953" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.286" x2="4.953" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-2.286" x2="4.953" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="2.286" x2="4.699" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.286" x2="2.794" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.397" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.762" x2="2.413" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.667" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-055X075" urn="urn:adsk.eagle:footprint:23146/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 5.5 x 7.5 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.794" x2="4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-2.794" x2="4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="2.794" x2="4.699" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.794" x2="2.794" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="2.032" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-2.032" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.762" x2="2.413" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-024X044" urn="urn:adsk.eagle:footprint:23147/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.159" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.159" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.159" y1="-0.381" x2="2.54" y2="0.381" layer="51"/>
<rectangle x1="-2.54" y1="-0.381" x2="-2.159" y2="0.381" layer="51"/>
</package>
<package name="C050-025X075" urn="urn:adsk.eagle:footprint:23148/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.016" x2="-3.683" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.27" x2="3.429" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.016" x2="3.683" y2="1.016" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="-3.429" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="3.683" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.27" x2="3.683" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.016" x2="-3.429" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.016" x2="-3.429" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-045X075" urn="urn:adsk.eagle:footprint:23149/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 4.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.032" x2="-3.683" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.286" x2="3.429" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.032" x2="3.683" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="-3.429" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="3.683" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.286" x2="3.683" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.032" x2="-3.429" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.032" x2="-3.429" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="2.667" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-030X075" urn="urn:adsk.eagle:footprint:23150/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 3 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.27" x2="-3.683" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.27" x2="3.683" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="3.683" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.524" x2="3.683" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.27" x2="-3.429" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.27" x2="-3.429" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-050X075" urn="urn:adsk.eagle:footprint:23151/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.286" x2="-3.683" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.54" x2="3.429" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.286" x2="3.683" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="-3.429" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="3.683" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.54" x2="3.683" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.286" x2="-3.429" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.286" x2="-3.429" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-055X075" urn="urn:adsk.eagle:footprint:23152/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 5.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.54" x2="-3.683" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.794" x2="3.429" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.54" x2="3.683" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="-3.429" y2="2.794" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="3.683" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.794" x2="3.683" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.54" x2="-3.429" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.54" x2="-3.429" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.302" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-075X075" urn="urn:adsk.eagle:footprint:23153/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 7.5 x 7.5 mm</description>
<wire x1="-1.524" y1="0" x2="-0.4572" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="0.762" width="0.4064" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0.762" x2="0.4318" y2="0" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.4318" y1="0" x2="0.4318" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="3.429" x2="-3.683" y2="-3.429" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-3.683" x2="3.429" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-3.429" x2="3.683" y2="3.429" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="-3.429" y2="3.683" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="3.683" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-3.683" x2="3.683" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-3.429" x2="-3.429" y2="-3.683" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="3.429" x2="-3.429" y2="3.683" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="4.064" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050H075X075" urn="urn:adsk.eagle:footprint:23154/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Horizontal, grid 5 mm, outline 7.5 x 7.5 mm</description>
<wire x1="-3.683" y1="7.112" x2="-3.683" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="0.508" x2="-3.302" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="0.508" x2="-1.778" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="0.508" x2="1.778" y2="0.508" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.508" x2="3.302" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.302" y1="0.508" x2="3.683" y2="0.508" width="0.1524" layer="21"/>
<wire x1="3.683" y1="0.508" x2="3.683" y2="7.112" width="0.1524" layer="21"/>
<wire x1="3.175" y1="7.62" x2="-3.175" y2="7.62" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="2.413" x2="-0.3048" y2="1.778" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="1.778" x2="-0.3048" y2="1.143" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="1.778" x2="-1.651" y2="1.778" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="2.413" x2="0.3302" y2="1.778" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="1.778" x2="0.3302" y2="1.143" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="1.778" x2="1.651" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="7.112" x2="-3.175" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.175" y1="7.62" x2="3.683" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="0" x2="-2.54" y2="0.254" width="0.508" layer="51"/>
<wire x1="2.54" y1="0" x2="2.54" y2="0.254" width="0.508" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.302" y="8.001" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="3.175" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.794" y1="0.127" x2="-2.286" y2="0.508" layer="51"/>
<rectangle x1="2.286" y1="0.127" x2="2.794" y2="0.508" layer="51"/>
</package>
<package name="C075-032X103" urn="urn:adsk.eagle:footprint:23155/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 3.2 x 10.3 mm</description>
<wire x1="4.826" y1="1.524" x2="-4.826" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-1.524" x2="4.826" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="1.524" x2="5.08" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-1.524" x2="5.08" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.27" x2="-4.826" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.27" x2="-4.826" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="0.508" y1="0" x2="2.54" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0" x2="-0.508" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="0.889" x2="-0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="-0.508" y1="0" x2="-0.508" y2="-0.889" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0.889" x2="0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-0.889" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.826" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-042X103" urn="urn:adsk.eagle:footprint:23156/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 4.2 x 10.3 mm</description>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.778" x2="-5.08" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.778" x2="5.08" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="5.08" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-2.032" x2="5.08" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.778" x2="-4.826" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.778" x2="-4.826" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.699" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-052X106" urn="urn:adsk.eagle:footprint:23157/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 5.2 x 10.6 mm</description>
<wire x1="4.953" y1="2.54" x2="-4.953" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.286" x2="-5.207" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.286" x2="5.207" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.54" x2="5.207" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-2.54" x2="5.207" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.286" x2="-4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.286" x2="-4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-043X133" urn="urn:adsk.eagle:footprint:23158/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 4.3 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.032" x2="6.096" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.604" y1="1.524" x2="6.604" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.032" x2="-6.096" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-1.524" x2="-6.604" y2="1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.032" x2="6.604" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.032" x2="6.604" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-1.524" x2="-6.096" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="1.524" x2="-6.096" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-054X133" urn="urn:adsk.eagle:footprint:23159/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 5.4 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.54" x2="6.096" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.032" x2="6.604" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.54" x2="-6.096" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.032" x2="-6.604" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.54" x2="6.604" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.54" x2="6.604" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.032" x2="-6.096" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.032" x2="-6.096" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.905" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-064X133" urn="urn:adsk.eagle:footprint:23160/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 6.4 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.096" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.54" x2="6.604" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="6.604" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-3.048" x2="6.604" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102_152-062X184" urn="urn:adsk.eagle:footprint:23161/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm + 15.2 mm, outline 6.2 x 18.4 mm</description>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="3.683" y2="0" width="0.1524" layer="21"/>
<wire x1="6.477" y1="0" x2="8.636" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.223" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.223" y1="3.048" x2="6.731" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.223" y1="-3.048" x2="6.731" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="2.54" x2="6.731" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="11.176" y1="3.048" x2="11.684" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="11.176" y1="-3.048" x2="11.684" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="11.176" y1="-3.048" x2="7.112" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="7.112" y1="3.048" x2="11.176" y2="3.048" width="0.1524" layer="21"/>
<wire x1="11.684" y1="2.54" x2="11.684" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="3" x="10.033" y="0" drill="1.016" shape="octagon"/>
<text x="-5.969" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-054X183" urn="urn:adsk.eagle:footprint:23162/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 5.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.032" x2="9.017" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-2.54" x2="-8.509" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.032" x2="-9.017" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="2.54" x2="8.509" y2="2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="2.54" x2="9.017" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-2.54" x2="9.017" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.032" x2="-8.509" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.032" x2="-8.509" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-064X183" urn="urn:adsk.eagle:footprint:23163/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 6.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.54" x2="9.017" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.048" x2="-8.509" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.54" x2="-9.017" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.048" x2="8.509" y2="3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.048" x2="9.017" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.048" x2="9.017" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.54" x2="-8.509" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.54" x2="-8.509" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-072X183" urn="urn:adsk.eagle:footprint:23164/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 7.2 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.048" x2="9.017" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.556" x2="-8.509" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.048" x2="-9.017" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.556" x2="8.509" y2="3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.556" x2="9.017" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.556" x2="9.017" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.048" x2="-8.509" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.048" x2="-8.509" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="3.937" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-084X183" urn="urn:adsk.eagle:footprint:23165/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 8.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.556" x2="9.017" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.064" x2="-8.509" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.556" x2="-9.017" y2="3.556" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.064" x2="8.509" y2="4.064" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.064" x2="9.017" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.064" x2="9.017" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.556" x2="-8.509" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.556" x2="-8.509" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="4.445" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-091X182" urn="urn:adsk.eagle:footprint:23166/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 9.1 x 18.2 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.937" x2="9.017" y2="-3.937" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.445" x2="-8.509" y2="-4.445" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.937" x2="-9.017" y2="3.937" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.445" x2="8.509" y2="4.445" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.445" x2="9.017" y2="3.937" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.445" x2="9.017" y2="-3.937" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.937" x2="-8.509" y2="-4.445" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.937" x2="-8.509" y2="4.445" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="4.826" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-062X268" urn="urn:adsk.eagle:footprint:23167/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 6.2 x 26.8 mm</description>
<wire x1="-12.827" y1="3.048" x2="12.827" y2="3.048" width="0.1524" layer="21"/>
<wire x1="13.335" y1="2.54" x2="13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.048" x2="-12.827" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-2.54" x2="-13.335" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.048" x2="13.335" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.048" x2="13.335" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-2.54" x2="-12.827" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="2.54" x2="-12.827" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.7" y="3.429" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-074X268" urn="urn:adsk.eagle:footprint:23168/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 7.4 x 26.8 mm</description>
<wire x1="-12.827" y1="3.556" x2="12.827" y2="3.556" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.048" x2="13.335" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.556" x2="-12.827" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.048" x2="-13.335" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.556" x2="13.335" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.556" x2="13.335" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.048" x2="-12.827" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.048" x2="-12.827" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.827" y="3.937" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-087X268" urn="urn:adsk.eagle:footprint:23169/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 8.7 x 26.8 mm</description>
<wire x1="-12.827" y1="4.318" x2="12.827" y2="4.318" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.81" x2="13.335" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-4.318" x2="-12.827" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.81" x2="-13.335" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="4.318" x2="13.335" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-4.318" x2="13.335" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.81" x2="-12.827" y2="-4.318" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.81" x2="-12.827" y2="4.318" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.827" y="4.699" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-108X268" urn="urn:adsk.eagle:footprint:23170/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 10.8 x 26.8 mm</description>
<wire x1="-12.827" y1="5.334" x2="12.827" y2="5.334" width="0.1524" layer="21"/>
<wire x1="13.335" y1="4.826" x2="13.335" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.334" x2="-12.827" y2="-5.334" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-4.826" x2="-13.335" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.334" x2="13.335" y2="4.826" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.334" x2="13.335" y2="-4.826" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-4.826" x2="-12.827" y2="-5.334" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="4.826" x2="-12.827" y2="5.334" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.954" y="5.715" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-113X268" urn="urn:adsk.eagle:footprint:23171/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 11.3 x 26.8 mm</description>
<wire x1="-12.827" y1="5.588" x2="12.827" y2="5.588" width="0.1524" layer="21"/>
<wire x1="13.335" y1="5.08" x2="13.335" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.588" x2="-12.827" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-5.08" x2="-13.335" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.588" x2="13.335" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.588" x2="13.335" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-5.08" x2="-12.827" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="5.08" x2="-12.827" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.954" y="5.969" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-093X316" urn="urn:adsk.eagle:footprint:23172/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 9.3 x 31.6 mm</description>
<wire x1="-15.24" y1="4.572" x2="15.24" y2="4.572" width="0.1524" layer="21"/>
<wire x1="15.748" y1="4.064" x2="15.748" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-4.572" x2="-15.24" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-4.064" x2="-15.748" y2="4.064" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="4.572" x2="15.748" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-4.572" x2="15.748" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-4.064" x2="-15.24" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="4.064" x2="-15.24" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="4.953" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-113X316" urn="urn:adsk.eagle:footprint:23173/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 11.3 x 31.6 mm</description>
<wire x1="-15.24" y1="5.588" x2="15.24" y2="5.588" width="0.1524" layer="21"/>
<wire x1="15.748" y1="5.08" x2="15.748" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-5.588" x2="-15.24" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-5.08" x2="-15.748" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="5.588" x2="15.748" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-5.588" x2="15.748" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-5.08" x2="-15.24" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="5.08" x2="-15.24" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="5.969" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-134X316" urn="urn:adsk.eagle:footprint:23174/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 13.4 x 31.6 mm</description>
<wire x1="-15.24" y1="6.604" x2="15.24" y2="6.604" width="0.1524" layer="21"/>
<wire x1="15.748" y1="6.096" x2="15.748" y2="-6.096" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-6.604" x2="-15.24" y2="-6.604" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-6.096" x2="-15.748" y2="6.096" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="6.604" x2="15.748" y2="6.096" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-6.604" x2="15.748" y2="-6.096" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-6.096" x2="-15.24" y2="-6.604" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="6.096" x2="-15.24" y2="6.604" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="6.985" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-205X316" urn="urn:adsk.eagle:footprint:23175/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 20.5 x 31.6 mm</description>
<wire x1="-15.24" y1="10.16" x2="15.24" y2="10.16" width="0.1524" layer="21"/>
<wire x1="15.748" y1="9.652" x2="15.748" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-10.16" x2="-15.24" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-9.652" x2="-15.748" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="10.16" x2="15.748" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-10.16" x2="15.748" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-9.652" x2="-15.24" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="9.652" x2="-15.24" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="10.541" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-4.318" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-137X374" urn="urn:adsk.eagle:footprint:23176/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 13.7 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="6.731" x2="18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="6.731" x2="-18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-6.731" x2="18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="18.542" y1="6.731" x2="-18.542" y2="6.731" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.2372" y="7.0612" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-162X374" urn="urn:adsk.eagle:footprint:23177/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 16.2 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="8.001" x2="18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="8.001" x2="-18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-8.001" x2="18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="18.542" y1="8.001" x2="-18.542" y2="8.001" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.3642" y="8.3312" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-182X374" urn="urn:adsk.eagle:footprint:23178/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 18.2 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="9.017" x2="18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="9.017" x2="-18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-9.017" x2="18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="18.542" y1="9.017" x2="-18.542" y2="9.017" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.3642" y="9.3472" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-192X418" urn="urn:adsk.eagle:footprint:23179/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 19.2 x 41.8 mm</description>
<wire x1="-20.32" y1="8.509" x2="20.32" y2="8.509" width="0.1524" layer="21"/>
<wire x1="20.828" y1="8.001" x2="20.828" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-8.509" x2="-20.32" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-8.001" x2="-20.828" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="8.509" x2="20.828" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-8.509" x2="20.828" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-8.001" x2="-20.32" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="8.001" x2="-20.32" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.447" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-203X418" urn="urn:adsk.eagle:footprint:23180/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 20.3 x 41.8 mm</description>
<wire x1="-20.32" y1="10.16" x2="20.32" y2="10.16" width="0.1524" layer="21"/>
<wire x1="20.828" y1="9.652" x2="20.828" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-10.16" x2="-20.32" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-9.652" x2="-20.828" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="10.16" x2="20.828" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-10.16" x2="20.828" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-9.652" x2="-20.32" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="9.652" x2="-20.32" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.32" y="10.541" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-035X075" urn="urn:adsk.eagle:footprint:23181/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 3.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.524" x2="-3.683" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.778" x2="3.429" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.524" x2="3.683" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="-3.429" y2="1.778" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="3.683" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.778" x2="3.683" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.524" x2="-3.429" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.524" x2="-3.429" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-155X418" urn="urn:adsk.eagle:footprint:23182/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 15.5 x 41.8 mm</description>
<wire x1="-20.32" y1="7.62" x2="20.32" y2="7.62" width="0.1524" layer="21"/>
<wire x1="20.828" y1="7.112" x2="20.828" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-7.62" x2="-20.32" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-7.112" x2="-20.828" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="7.62" x2="20.828" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-7.62" x2="20.828" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-7.112" x2="-20.32" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="7.112" x2="-20.32" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.447" y="8.001" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-063X106" urn="urn:adsk.eagle:footprint:23183/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 6.3 x 10.6 mm</description>
<wire x1="4.953" y1="3.048" x2="-4.953" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.794" x2="-5.207" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-3.048" x2="4.953" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.794" x2="5.207" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="3.048" x2="5.207" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-3.048" x2="5.207" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.794" x2="-4.953" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.794" x2="-4.953" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-154X316" urn="urn:adsk.eagle:footprint:23184/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 15.4 x 31.6 mm</description>
<wire x1="-15.24" y1="7.62" x2="15.24" y2="7.62" width="0.1524" layer="21"/>
<wire x1="15.748" y1="7.112" x2="15.748" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-7.62" x2="-15.24" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-7.112" x2="-15.748" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="7.62" x2="15.748" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-7.62" x2="15.748" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-7.112" x2="-15.24" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="7.112" x2="-15.24" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="8.001" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-173X316" urn="urn:adsk.eagle:footprint:23185/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 17.3 x 31.6 mm</description>
<wire x1="-15.24" y1="8.509" x2="15.24" y2="8.509" width="0.1524" layer="21"/>
<wire x1="15.748" y1="8.001" x2="15.748" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-8.509" x2="-15.24" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-8.001" x2="-15.748" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="8.509" x2="15.748" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-8.509" x2="15.748" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-8.001" x2="-15.24" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="8.001" x2="-15.24" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C0402K" urn="urn:adsk.eagle:footprint:23186/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0204 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 1005</description>
<wire x1="-0.425" y1="0.2" x2="0.425" y2="0.2" width="0.1016" layer="51"/>
<wire x1="0.425" y1="-0.2" x2="-0.425" y2="-0.2" width="0.1016" layer="51"/>
<smd name="1" x="-0.6" y="0" dx="0.925" dy="0.74" layer="1"/>
<smd name="2" x="0.6" y="0" dx="0.925" dy="0.74" layer="1"/>
<text x="-0.5" y="0.425" size="1.016" layer="25">&gt;NAME</text>
<text x="-0.5" y="-1.45" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-0.5" y1="-0.25" x2="-0.225" y2="0.25" layer="51"/>
<rectangle x1="0.225" y1="-0.25" x2="0.5" y2="0.25" layer="51"/>
</package>
<package name="C0603K" urn="urn:adsk.eagle:footprint:23187/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0603 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 1608</description>
<wire x1="-0.725" y1="0.35" x2="0.725" y2="0.35" width="0.1016" layer="51"/>
<wire x1="0.725" y1="-0.35" x2="-0.725" y2="-0.35" width="0.1016" layer="51"/>
<smd name="1" x="-0.875" y="0" dx="1.05" dy="1.08" layer="1"/>
<smd name="2" x="0.875" y="0" dx="1.05" dy="1.08" layer="1"/>
<text x="-0.8" y="0.65" size="1.016" layer="25">&gt;NAME</text>
<text x="-0.8" y="-1.65" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8" y1="-0.4" x2="-0.45" y2="0.4" layer="51"/>
<rectangle x1="0.45" y1="-0.4" x2="0.8" y2="0.4" layer="51"/>
</package>
<package name="C0805K" urn="urn:adsk.eagle:footprint:23188/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0805 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 2012</description>
<wire x1="-0.925" y1="0.6" x2="0.925" y2="0.6" width="0.1016" layer="51"/>
<wire x1="0.925" y1="-0.6" x2="-0.925" y2="-0.6" width="0.1016" layer="51"/>
<smd name="1" x="-1" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="1" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1" y="0.875" size="1.016" layer="25">&gt;NAME</text>
<text x="-1" y="-1.9" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1" y1="-0.65" x2="-0.5" y2="0.65" layer="51"/>
<rectangle x1="0.5" y1="-0.65" x2="1" y2="0.65" layer="51"/>
</package>
<package name="C1206K" urn="urn:adsk.eagle:footprint:23189/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1206 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 3216</description>
<wire x1="-1.525" y1="0.75" x2="1.525" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-0.75" x2="-1.525" y2="-0.75" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.5" dy="2" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.5" dy="2" layer="1"/>
<text x="-1.6" y="1.1" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-2.1" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-0.8" x2="-1.1" y2="0.8" layer="51"/>
<rectangle x1="1.1" y1="-0.8" x2="1.6" y2="0.8" layer="51"/>
</package>
<package name="C1210K" urn="urn:adsk.eagle:footprint:23190/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1210 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 3225</description>
<wire x1="-1.525" y1="1.175" x2="1.525" y2="1.175" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-1.175" x2="-1.525" y2="-1.175" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.5" dy="2.9" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.5" dy="2.9" layer="1"/>
<text x="-1.6" y="1.55" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-2.575" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-1.25" x2="-1.1" y2="1.25" layer="51"/>
<rectangle x1="1.1" y1="-1.25" x2="1.6" y2="1.25" layer="51"/>
</package>
<package name="C1812K" urn="urn:adsk.eagle:footprint:23191/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1812 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 4532</description>
<wire x1="-2.175" y1="1.525" x2="2.175" y2="1.525" width="0.1016" layer="51"/>
<wire x1="2.175" y1="-1.525" x2="-2.175" y2="-1.525" width="0.1016" layer="51"/>
<smd name="1" x="-2.05" y="0" dx="1.8" dy="3.7" layer="1"/>
<smd name="2" x="2.05" y="0" dx="1.8" dy="3.7" layer="1"/>
<text x="-2.25" y="1.95" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.25" y="-2.975" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.25" y1="-1.6" x2="-1.65" y2="1.6" layer="51"/>
<rectangle x1="1.65" y1="-1.6" x2="2.25" y2="1.6" layer="51"/>
</package>
<package name="C1825K" urn="urn:adsk.eagle:footprint:23192/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1825 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 4564</description>
<wire x1="-1.525" y1="3.125" x2="1.525" y2="3.125" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-3.125" x2="-1.525" y2="-3.125" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.8" dy="6.9" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.8" dy="6.9" layer="1"/>
<text x="-1.6" y="3.55" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-4.625" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-3.2" x2="-1.1" y2="3.2" layer="51"/>
<rectangle x1="1.1" y1="-3.2" x2="1.6" y2="3.2" layer="51"/>
</package>
<package name="C2220K" urn="urn:adsk.eagle:footprint:23193/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 2220 reflow solder&lt;/b&gt;&lt;p&gt;Metric Code Size 5650</description>
<wire x1="-2.725" y1="2.425" x2="2.725" y2="2.425" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-2.425" x2="-2.725" y2="-2.425" width="0.1016" layer="51"/>
<smd name="1" x="-2.55" y="0" dx="1.85" dy="5.5" layer="1"/>
<smd name="2" x="2.55" y="0" dx="1.85" dy="5.5" layer="1"/>
<text x="-2.8" y="2.95" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.8" y="-3.975" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-2.5" x2="-2.2" y2="2.5" layer="51"/>
<rectangle x1="2.2" y1="-2.5" x2="2.8" y2="2.5" layer="51"/>
</package>
<package name="C2225K" urn="urn:adsk.eagle:footprint:23194/1" library_version="10">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 2225 reflow solder&lt;/b&gt;&lt;p&gt;Metric Code Size 5664</description>
<wire x1="-2.725" y1="3.075" x2="2.725" y2="3.075" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-3.075" x2="-2.725" y2="-3.075" width="0.1016" layer="51"/>
<smd name="1" x="-2.55" y="0" dx="1.85" dy="6.8" layer="1"/>
<smd name="2" x="2.55" y="0" dx="1.85" dy="6.8" layer="1"/>
<text x="-2.8" y="3.6" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.8" y="-4.575" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-3.15" x2="-2.2" y2="3.15" layer="51"/>
<rectangle x1="2.2" y1="-3.15" x2="2.8" y2="3.15" layer="51"/>
</package>
<package name="HPC0201" urn="urn:adsk.eagle:footprint:25783/1" library_version="10">
<description>&lt;b&gt; &lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com/docs/10129/hpc0201a.pdf</description>
<smd name="1" x="-0.18" y="0" dx="0.2" dy="0.35" layer="1"/>
<smd name="2" x="0.18" y="0" dx="0.2" dy="0.35" layer="1"/>
<text x="-0.75" y="0.74" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.785" y="-1.865" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.305" y1="-0.15" x2="0.305" y2="0.15" layer="51"/>
</package>
<package name="C0201" urn="urn:adsk.eagle:footprint:23196/1" library_version="10">
<description>Source: http://www.avxcorp.com/docs/catalogs/cx5r.pdf</description>
<smd name="1" x="-0.25" y="0" dx="0.25" dy="0.35" layer="1"/>
<smd name="2" x="0.25" y="0" dx="0.25" dy="0.35" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="0.1" x2="0.15" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="-0.1" layer="51"/>
</package>
<package name="C1808" urn="urn:adsk.eagle:footprint:23197/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Source: AVX .. aphvc.pdf</description>
<wire x1="-1.4732" y1="0.9502" x2="1.4732" y2="0.9502" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-0.9502" x2="1.4732" y2="-0.9502" width="0.1016" layer="51"/>
<smd name="1" x="-1.95" y="0" dx="1.6" dy="2.2" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.6" dy="2.2" layer="1"/>
<text x="-2.233" y="1.827" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.233" y="-2.842" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.275" y1="-1.015" x2="-1.225" y2="1.015" layer="51"/>
<rectangle x1="1.225" y1="-1.015" x2="2.275" y2="1.015" layer="51"/>
</package>
<package name="C3640" urn="urn:adsk.eagle:footprint:23198/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Source: AVX .. aphvc.pdf</description>
<wire x1="-3.8322" y1="5.0496" x2="3.8322" y2="5.0496" width="0.1016" layer="51"/>
<wire x1="-3.8322" y1="-5.0496" x2="3.8322" y2="-5.0496" width="0.1016" layer="51"/>
<smd name="1" x="-4.267" y="0" dx="2.6" dy="10.7" layer="1"/>
<smd name="2" x="4.267" y="0" dx="2.6" dy="10.7" layer="1"/>
<text x="-4.647" y="6.465" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.647" y="-7.255" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-4.57" y1="-5.1" x2="-3.05" y2="5.1" layer="51"/>
<rectangle x1="3.05" y1="-5.1" x2="4.5688" y2="5.1" layer="51"/>
</package>
<package name="C01005" urn="urn:adsk.eagle:footprint:23199/1" library_version="10">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="CAPC1005X60" urn="urn:adsk.eagle:package:23626/2" type="model" library_version="10">
<description>Chip, 1.00 X 0.50 X 0.60 mm body
&lt;p&gt;Chip package with body size 1.00 X 0.50 X 0.60 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="C0402"/>
</packageinstances>
</package3d>
<package3d name="C0504" urn="urn:adsk.eagle:package:23624/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0504"/>
</packageinstances>
</package3d>
<package3d name="C0603" urn="urn:adsk.eagle:package:23616/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0603"/>
</packageinstances>
</package3d>
<package3d name="C0805" urn="urn:adsk.eagle:package:23617/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0805"/>
</packageinstances>
</package3d>
<package3d name="C1206" urn="urn:adsk.eagle:package:23618/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1206"/>
</packageinstances>
</package3d>
<package3d name="C1210" urn="urn:adsk.eagle:package:23619/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1210"/>
</packageinstances>
</package3d>
<package3d name="C1310" urn="urn:adsk.eagle:package:23620/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1310"/>
</packageinstances>
</package3d>
<package3d name="C1608" urn="urn:adsk.eagle:package:23621/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1608"/>
</packageinstances>
</package3d>
<package3d name="C1812" urn="urn:adsk.eagle:package:23622/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1812"/>
</packageinstances>
</package3d>
<package3d name="C1825" urn="urn:adsk.eagle:package:23623/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1825"/>
</packageinstances>
</package3d>
<package3d name="C2012" urn="urn:adsk.eagle:package:23625/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C2012"/>
</packageinstances>
</package3d>
<package3d name="C3216" urn="urn:adsk.eagle:package:23628/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C3216"/>
</packageinstances>
</package3d>
<package3d name="C3225" urn="urn:adsk.eagle:package:23655/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C3225"/>
</packageinstances>
</package3d>
<package3d name="C4532" urn="urn:adsk.eagle:package:23627/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C4532"/>
</packageinstances>
</package3d>
<package3d name="C4564" urn="urn:adsk.eagle:package:23648/2" type="model" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C4564"/>
</packageinstances>
</package3d>
<package3d name="C025-024X044" urn="urn:adsk.eagle:package:23630/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 mm, outline 2.4 x 4.4 mm</description>
<packageinstances>
<packageinstance name="C025-024X044"/>
</packageinstances>
</package3d>
<package3d name="C025-025X050" urn="urn:adsk.eagle:package:23629/2" type="model" library_version="10">
<description>CAPACITOR
grid 2.5 mm, outline 2.5 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-025X050"/>
</packageinstances>
</package3d>
<package3d name="C025-030X050" urn="urn:adsk.eagle:package:23631/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 mm, outline 3 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-030X050"/>
</packageinstances>
</package3d>
<package3d name="C025-040X050" urn="urn:adsk.eagle:package:23634/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 mm, outline 4 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-040X050"/>
</packageinstances>
</package3d>
<package3d name="C025-050X050" urn="urn:adsk.eagle:package:23633/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 mm, outline 5 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-050X050"/>
</packageinstances>
</package3d>
<package3d name="C025-060X050" urn="urn:adsk.eagle:package:23632/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 mm, outline 6 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-060X050"/>
</packageinstances>
</package3d>
<package3d name="C025_050-024X070" urn="urn:adsk.eagle:package:23639/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 mm + 5 mm, outline 2.4 x 7 mm</description>
<packageinstances>
<packageinstance name="C025_050-024X070"/>
</packageinstances>
</package3d>
<package3d name="C025_050-025X075" urn="urn:adsk.eagle:package:23641/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 2.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-025X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-035X075" urn="urn:adsk.eagle:package:23651/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 3.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-035X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-045X075" urn="urn:adsk.eagle:package:23635/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 4.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-045X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-055X075" urn="urn:adsk.eagle:package:23636/1" type="box" library_version="10">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 5.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-055X075"/>
</packageinstances>
</package3d>
<package3d name="C050-024X044" urn="urn:adsk.eagle:package:23643/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 2.4 x 4.4 mm</description>
<packageinstances>
<packageinstance name="C050-024X044"/>
</packageinstances>
</package3d>
<package3d name="C050-025X075" urn="urn:adsk.eagle:package:23637/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 2.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-025X075"/>
</packageinstances>
</package3d>
<package3d name="C050-045X075" urn="urn:adsk.eagle:package:23638/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 4.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-045X075"/>
</packageinstances>
</package3d>
<package3d name="C050-030X075" urn="urn:adsk.eagle:package:23640/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 3 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-030X075"/>
</packageinstances>
</package3d>
<package3d name="C050-050X075" urn="urn:adsk.eagle:package:23665/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-050X075"/>
</packageinstances>
</package3d>
<package3d name="C050-055X075" urn="urn:adsk.eagle:package:23642/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 5.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-055X075"/>
</packageinstances>
</package3d>
<package3d name="C050-075X075" urn="urn:adsk.eagle:package:23645/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 7.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-075X075"/>
</packageinstances>
</package3d>
<package3d name="C050H075X075" urn="urn:adsk.eagle:package:23644/1" type="box" library_version="10">
<description>CAPACITOR
Horizontal, grid 5 mm, outline 7.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050H075X075"/>
</packageinstances>
</package3d>
<package3d name="C075-032X103" urn="urn:adsk.eagle:package:23646/1" type="box" library_version="10">
<description>CAPACITOR
grid 7.5 mm, outline 3.2 x 10.3 mm</description>
<packageinstances>
<packageinstance name="C075-032X103"/>
</packageinstances>
</package3d>
<package3d name="C075-042X103" urn="urn:adsk.eagle:package:23656/1" type="box" library_version="10">
<description>CAPACITOR
grid 7.5 mm, outline 4.2 x 10.3 mm</description>
<packageinstances>
<packageinstance name="C075-042X103"/>
</packageinstances>
</package3d>
<package3d name="C075-052X106" urn="urn:adsk.eagle:package:23650/1" type="box" library_version="10">
<description>CAPACITOR
grid 7.5 mm, outline 5.2 x 10.6 mm</description>
<packageinstances>
<packageinstance name="C075-052X106"/>
</packageinstances>
</package3d>
<package3d name="C102-043X133" urn="urn:adsk.eagle:package:23647/1" type="box" library_version="10">
<description>CAPACITOR
grid 10.2 mm, outline 4.3 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-043X133"/>
</packageinstances>
</package3d>
<package3d name="C102-054X133" urn="urn:adsk.eagle:package:23649/1" type="box" library_version="10">
<description>CAPACITOR
grid 10.2 mm, outline 5.4 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-054X133"/>
</packageinstances>
</package3d>
<package3d name="C102-064X133" urn="urn:adsk.eagle:package:23653/1" type="box" library_version="10">
<description>CAPACITOR
grid 10.2 mm, outline 6.4 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-064X133"/>
</packageinstances>
</package3d>
<package3d name="C102_152-062X184" urn="urn:adsk.eagle:package:23652/1" type="box" library_version="10">
<description>CAPACITOR
grid 10.2 mm + 15.2 mm, outline 6.2 x 18.4 mm</description>
<packageinstances>
<packageinstance name="C102_152-062X184"/>
</packageinstances>
</package3d>
<package3d name="C150-054X183" urn="urn:adsk.eagle:package:23669/1" type="box" library_version="10">
<description>CAPACITOR
grid 15 mm, outline 5.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-054X183"/>
</packageinstances>
</package3d>
<package3d name="C150-064X183" urn="urn:adsk.eagle:package:23654/1" type="box" library_version="10">
<description>CAPACITOR
grid 15 mm, outline 6.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-064X183"/>
</packageinstances>
</package3d>
<package3d name="C150-072X183" urn="urn:adsk.eagle:package:23657/1" type="box" library_version="10">
<description>CAPACITOR
grid 15 mm, outline 7.2 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-072X183"/>
</packageinstances>
</package3d>
<package3d name="C150-084X183" urn="urn:adsk.eagle:package:23658/1" type="box" library_version="10">
<description>CAPACITOR
grid 15 mm, outline 8.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-084X183"/>
</packageinstances>
</package3d>
<package3d name="C150-091X182" urn="urn:adsk.eagle:package:23659/1" type="box" library_version="10">
<description>CAPACITOR
grid 15 mm, outline 9.1 x 18.2 mm</description>
<packageinstances>
<packageinstance name="C150-091X182"/>
</packageinstances>
</package3d>
<package3d name="C225-062X268" urn="urn:adsk.eagle:package:23661/1" type="box" library_version="10">
<description>CAPACITOR
grid 22.5 mm, outline 6.2 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-062X268"/>
</packageinstances>
</package3d>
<package3d name="C225-074X268" urn="urn:adsk.eagle:package:23660/1" type="box" library_version="10">
<description>CAPACITOR
grid 22.5 mm, outline 7.4 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-074X268"/>
</packageinstances>
</package3d>
<package3d name="C225-087X268" urn="urn:adsk.eagle:package:23662/1" type="box" library_version="10">
<description>CAPACITOR
grid 22.5 mm, outline 8.7 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-087X268"/>
</packageinstances>
</package3d>
<package3d name="C225-108X268" urn="urn:adsk.eagle:package:23663/1" type="box" library_version="10">
<description>CAPACITOR
grid 22.5 mm, outline 10.8 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-108X268"/>
</packageinstances>
</package3d>
<package3d name="C225-113X268" urn="urn:adsk.eagle:package:23667/1" type="box" library_version="10">
<description>CAPACITOR
grid 22.5 mm, outline 11.3 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-113X268"/>
</packageinstances>
</package3d>
<package3d name="C275-093X316" urn="urn:adsk.eagle:package:23701/1" type="box" library_version="10">
<description>CAPACITOR
grid 27.5 mm, outline 9.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-093X316"/>
</packageinstances>
</package3d>
<package3d name="C275-113X316" urn="urn:adsk.eagle:package:23673/1" type="box" library_version="10">
<description>CAPACITOR
grid 27.5 mm, outline 11.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-113X316"/>
</packageinstances>
</package3d>
<package3d name="C275-134X316" urn="urn:adsk.eagle:package:23664/1" type="box" library_version="10">
<description>CAPACITOR
grid 27.5 mm, outline 13.4 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-134X316"/>
</packageinstances>
</package3d>
<package3d name="C275-205X316" urn="urn:adsk.eagle:package:23666/1" type="box" library_version="10">
<description>CAPACITOR
grid 27.5 mm, outline 20.5 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-205X316"/>
</packageinstances>
</package3d>
<package3d name="C325-137X374" urn="urn:adsk.eagle:package:23672/1" type="box" library_version="10">
<description>CAPACITOR
grid 32.5 mm, outline 13.7 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-137X374"/>
</packageinstances>
</package3d>
<package3d name="C325-162X374" urn="urn:adsk.eagle:package:23670/1" type="box" library_version="10">
<description>CAPACITOR
grid 32.5 mm, outline 16.2 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-162X374"/>
</packageinstances>
</package3d>
<package3d name="C325-182X374" urn="urn:adsk.eagle:package:23668/1" type="box" library_version="10">
<description>CAPACITOR
grid 32.5 mm, outline 18.2 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-182X374"/>
</packageinstances>
</package3d>
<package3d name="C375-192X418" urn="urn:adsk.eagle:package:23674/1" type="box" library_version="10">
<description>CAPACITOR
grid 37.5 mm, outline 19.2 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-192X418"/>
</packageinstances>
</package3d>
<package3d name="C375-203X418" urn="urn:adsk.eagle:package:23671/1" type="box" library_version="10">
<description>CAPACITOR
grid 37.5 mm, outline 20.3 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-203X418"/>
</packageinstances>
</package3d>
<package3d name="C050-035X075" urn="urn:adsk.eagle:package:23677/1" type="box" library_version="10">
<description>CAPACITOR
grid 5 mm, outline 3.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-035X075"/>
</packageinstances>
</package3d>
<package3d name="C375-155X418" urn="urn:adsk.eagle:package:23675/1" type="box" library_version="10">
<description>CAPACITOR
grid 37.5 mm, outline 15.5 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-155X418"/>
</packageinstances>
</package3d>
<package3d name="C075-063X106" urn="urn:adsk.eagle:package:23678/1" type="box" library_version="10">
<description>CAPACITOR
grid 7.5 mm, outline 6.3 x 10.6 mm</description>
<packageinstances>
<packageinstance name="C075-063X106"/>
</packageinstances>
</package3d>
<package3d name="C275-154X316" urn="urn:adsk.eagle:package:23685/1" type="box" library_version="10">
<description>CAPACITOR
grid 27.5 mm, outline 15.4 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-154X316"/>
</packageinstances>
</package3d>
<package3d name="C275-173X316" urn="urn:adsk.eagle:package:23676/1" type="box" library_version="10">
<description>CAPACITOR
grid 27.5 mm, outline 17.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-173X316"/>
</packageinstances>
</package3d>
<package3d name="C0402K" urn="urn:adsk.eagle:package:23679/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 0204 reflow solder
Metric Code Size 1005</description>
<packageinstances>
<packageinstance name="C0402K"/>
</packageinstances>
</package3d>
<package3d name="C0603K" urn="urn:adsk.eagle:package:23680/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 0603 reflow solder
Metric Code Size 1608</description>
<packageinstances>
<packageinstance name="C0603K"/>
</packageinstances>
</package3d>
<package3d name="C0805K" urn="urn:adsk.eagle:package:23681/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 0805 reflow solder
Metric Code Size 2012</description>
<packageinstances>
<packageinstance name="C0805K"/>
</packageinstances>
</package3d>
<package3d name="C1206K" urn="urn:adsk.eagle:package:23682/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 1206 reflow solder
Metric Code Size 3216</description>
<packageinstances>
<packageinstance name="C1206K"/>
</packageinstances>
</package3d>
<package3d name="C1210K" urn="urn:adsk.eagle:package:23683/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 1210 reflow solder
Metric Code Size 3225</description>
<packageinstances>
<packageinstance name="C1210K"/>
</packageinstances>
</package3d>
<package3d name="C1812K" urn="urn:adsk.eagle:package:23686/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 1812 reflow solder
Metric Code Size 4532</description>
<packageinstances>
<packageinstance name="C1812K"/>
</packageinstances>
</package3d>
<package3d name="C1825K" urn="urn:adsk.eagle:package:23684/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 1825 reflow solder
Metric Code Size 4564</description>
<packageinstances>
<packageinstance name="C1825K"/>
</packageinstances>
</package3d>
<package3d name="C2220K" urn="urn:adsk.eagle:package:23687/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 2220 reflow solderMetric Code Size 5650</description>
<packageinstances>
<packageinstance name="C2220K"/>
</packageinstances>
</package3d>
<package3d name="C2225K" urn="urn:adsk.eagle:package:23692/2" type="model" library_version="10">
<description>Ceramic Chip Capacitor KEMET 2225 reflow solderMetric Code Size 5664</description>
<packageinstances>
<packageinstance name="C2225K"/>
</packageinstances>
</package3d>
<package3d name="HPC0201" urn="urn:adsk.eagle:package:26213/1" type="box" library_version="10">
<description> 
Source: http://www.vishay.com/docs/10129/hpc0201a.pdf</description>
<packageinstances>
<packageinstance name="HPC0201"/>
</packageinstances>
</package3d>
<package3d name="C0201" urn="urn:adsk.eagle:package:23690/2" type="model" library_version="10">
<description>Source: http://www.avxcorp.com/docs/catalogs/cx5r.pdf</description>
<packageinstances>
<packageinstance name="C0201"/>
</packageinstances>
</package3d>
<package3d name="C1808" urn="urn:adsk.eagle:package:23689/2" type="model" library_version="10">
<description>CAPACITOR
Source: AVX .. aphvc.pdf</description>
<packageinstances>
<packageinstance name="C1808"/>
</packageinstances>
</package3d>
<package3d name="C3640" urn="urn:adsk.eagle:package:23693/2" type="model" library_version="10">
<description>CAPACITOR
Source: AVX .. aphvc.pdf</description>
<packageinstances>
<packageinstance name="C3640"/>
</packageinstances>
</package3d>
<package3d name="C01005" urn="urn:adsk.eagle:package:23691/1" type="box" library_version="10">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C01005"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="C-EU" urn="urn:adsk.eagle:symbol:23120/1" library_version="10">
<wire x1="0" y1="0" x2="0" y2="-0.508" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="-2.032" width="0.1524" layer="94"/>
<text x="1.524" y="0.381" size="1.778" layer="95">&gt;NAME</text>
<text x="1.524" y="-4.699" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-2.032" x2="2.032" y2="-1.524" layer="94"/>
<rectangle x1="-2.032" y1="-1.016" x2="2.032" y2="-0.508" layer="94"/>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C-EU" urn="urn:adsk.eagle:component:23793/45" prefix="C" uservalue="yes" library_version="10">
<description>&lt;B&gt;CAPACITOR&lt;/B&gt;, European symbol</description>
<gates>
<gate name="G$1" symbol="C-EU" x="0" y="0"/>
</gates>
<devices>
<device name="C0402" package="C0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23626/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0504" package="C0504">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23624/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0603" package="C0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23616/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0805" package="C0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23617/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1206" package="C1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23618/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1210" package="C1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23619/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1310" package="C1310">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23620/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1608" package="C1608">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23621/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1812" package="C1812">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23622/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1825" package="C1825">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23623/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C2012" package="C2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23625/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C3216" package="C3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23628/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C3225" package="C3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23655/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C4532" package="C4532">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23627/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C4564" package="C4564">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23648/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-024X044" package="C025-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23630/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-025X050" package="C025-025X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23629/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-030X050" package="C025-030X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23631/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-040X050" package="C025-040X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23634/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-050X050" package="C025-050X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23633/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-060X050" package="C025-060X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23632/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C025_050-024X070" package="C025_050-024X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23639/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-025X075" package="C025_050-025X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23641/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-035X075" package="C025_050-035X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23651/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-045X075" package="C025_050-045X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23635/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-055X075" package="C025_050-055X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23636/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-024X044" package="C050-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23643/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-025X075" package="C050-025X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23637/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-045X075" package="C050-045X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23638/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-030X075" package="C050-030X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23640/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-050X075" package="C050-050X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23665/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-055X075" package="C050-055X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23642/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-075X075" package="C050-075X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23645/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050H075X075" package="C050H075X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23644/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-032X103" package="C075-032X103">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23646/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-042X103" package="C075-042X103">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23656/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-052X106" package="C075-052X106">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23650/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102-043X133" package="C102-043X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23647/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102-054X133" package="C102-054X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23649/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102-064X133" package="C102-064X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23653/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102_152-062X184" package="C102_152-062X184">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23652/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-054X183" package="C150-054X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23669/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-064X183" package="C150-064X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23654/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-072X183" package="C150-072X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23657/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-084X183" package="C150-084X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23658/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-091X182" package="C150-091X182">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23659/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-062X268" package="C225-062X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23661/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-074X268" package="C225-074X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23660/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-087X268" package="C225-087X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23662/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-108X268" package="C225-108X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23663/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-113X268" package="C225-113X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23667/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-093X316" package="C275-093X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23701/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-113X316" package="C275-113X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23673/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-134X316" package="C275-134X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23664/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-205X316" package="C275-205X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23666/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="325-137X374" package="C325-137X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23672/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="325-162X374" package="C325-162X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23670/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="325-182X374" package="C325-182X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23668/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="375-192X418" package="C375-192X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23674/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="375-203X418" package="C375-203X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23671/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-035X075" package="C050-035X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23677/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="375-155X418" package="C375-155X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23675/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-063X106" package="C075-063X106">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23678/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-154X316" package="C275-154X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23685/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-173X316" package="C275-173X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23676/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0402K" package="C0402K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23679/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0603K" package="C0603K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23680/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0805K" package="C0805K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23681/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1206K" package="C1206K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23682/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1210K" package="C1210K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23683/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1812K" package="C1812K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23686/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1825K" package="C1825K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23684/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C2220K" package="C2220K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23687/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C2225K" package="C2225K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23692/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="HPC0201" package="HPC0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26213/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0201" package="C0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23690/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1808" package="C1808">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23689/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C3640" package="C3640">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23693/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="01005" package="C01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23691/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
</devices>
<spice>
<pinmapping spiceprefix="C">
<pinmap gate="G$1" pin="1" pinorder="1"/>
<pinmap gate="G$1" pin="2" pinorder="2"/>
</pinmapping>
</spice>
</deviceset>
</devicesets>
</library>
<library name="diode" urn="urn:adsk.eagle:library:210">
<description>&lt;b&gt;Diodes&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Motorola : www.onsemi.com
&lt;li&gt;Fairchild : www.fairchildsemi.com
&lt;li&gt;Philips : www.semiconductors.com
&lt;li&gt;Vishay : www.vishay.de
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DO41-10" urn="urn:adsk.eagle:footprint:43094/1" library_version="7">
<description>&lt;B&gt;DIODE&lt;/B&gt;&lt;p&gt;
diameter 2.54 mm, horizontal, grid 10.16 mm</description>
<wire x1="2.032" y1="-1.27" x2="-2.032" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.032" y1="-1.27" x2="2.032" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.032" y1="1.27" x2="2.032" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.032" y1="1.27" x2="-2.032" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.762" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.762" layer="51"/>
<wire x1="-0.635" y1="0" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="1.016" y1="0.635" x2="1.016" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="1.016" y1="-0.635" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="1.016" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<pad name="A" x="5.08" y="0" drill="1.1176"/>
<pad name="C" x="-5.08" y="0" drill="1.1176"/>
<text x="-2.032" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.27" x2="-1.143" y2="1.27" layer="21"/>
<rectangle x1="2.032" y1="-0.381" x2="3.937" y2="0.381" layer="21"/>
<rectangle x1="-3.937" y1="-0.381" x2="-2.032" y2="0.381" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="DO41-10" urn="urn:adsk.eagle:package:43336/1" type="box" library_version="7">
<description>DIODE
diameter 2.54 mm, horizontal, grid 10.16 mm</description>
<packageinstances>
<packageinstance name="DO41-10"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="D" urn="urn:adsk.eagle:symbol:43091/2" library_version="7">
<wire x1="-1.27" y1="-1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="-1.27" y2="1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="-1.27" y1="0" x2="-2.54" y2="0" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<text x="2.54" y="0.4826" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-2.3114" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="0" size="0.4064" layer="99" align="center">SpiceOrder 1</text>
<text x="2.54" y="0" size="0.4064" layer="99" align="center">SpiceOrder 2</text>
<pin name="A" x="-2.54" y="0" visible="off" length="point" direction="pas"/>
<pin name="C" x="2.54" y="0" visible="off" length="point" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="1N4004" urn="urn:adsk.eagle:component:43462/2" prefix="D" library_version="7">
<description>&lt;B&gt;DIODE&lt;/B&gt;&lt;p&gt;
general purpose rectifier, 1 A</description>
<gates>
<gate name="1" symbol="D" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DO41-10">
<connects>
<connect gate="1" pin="A" pad="A"/>
<connect gate="1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:43336/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="linear" urn="urn:adsk.eagle:library:262">
<description>&lt;b&gt;Linear Devices&lt;/b&gt;&lt;p&gt;
Operational amplifiers,  comparators, voltage regulators, ADCs, DACs, etc.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="TO92" urn="urn:adsk.eagle:footprint:16150/1" library_version="4">
<description>&lt;b&gt;TO-92&lt;/b&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="-0.7869" y2="2.5484" width="0.1524" layer="21" curve="-111.097684"/>
<wire x1="0.7869" y1="2.5484" x2="2.095" y2="-1.651" width="0.1524" layer="21" curve="-111.097684"/>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-2.254" y1="-0.254" x2="-0.286" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-2.655" y1="-0.254" x2="-2.254" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="-0.286" y1="-0.254" x2="0.286" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="2.254" y1="-0.254" x2="2.655" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="0.286" y1="-0.254" x2="2.254" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-0.7864" y1="2.5484" x2="0.7864" y2="2.5484" width="0.1524" layer="51" curve="-34.298964"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="2.921" y="-1.27" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO220H" urn="urn:adsk.eagle:footprint:16151/1" library_version="4">
<description>&lt;b&gt;TO-220&lt;/b&gt;</description>
<wire x1="-5.207" y1="-7.62" x2="5.207" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="5.207" y1="8.255" x2="-5.207" y2="8.255" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-7.62" x2="5.207" y2="4.826" width="0.1524" layer="21"/>
<wire x1="5.207" y1="4.826" x2="4.318" y2="4.826" width="0.1524" layer="21"/>
<wire x1="4.318" y1="4.826" x2="4.318" y2="6.35" width="0.1524" layer="21"/>
<wire x1="4.318" y1="6.35" x2="5.207" y2="6.35" width="0.1524" layer="21"/>
<wire x1="5.207" y1="6.35" x2="5.207" y2="8.255" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-7.62" x2="-5.207" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="4.826" x2="-4.318" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="4.826" x2="-4.318" y2="6.35" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="6.35" x2="-5.207" y2="6.35" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="6.35" x2="-5.207" y2="8.255" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-6.985" x2="4.572" y2="-6.985" width="0.0508" layer="21"/>
<wire x1="4.572" y1="1.27" x2="4.572" y2="-6.985" width="0.0508" layer="21"/>
<wire x1="4.572" y1="1.27" x2="-4.572" y2="1.27" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-6.985" x2="-4.572" y2="1.27" width="0.0508" layer="21"/>
<circle x="0" y="4.826" radius="1.8034" width="0.1524" layer="21"/>
<circle x="0" y="4.826" radius="2.54" width="0" layer="43"/>
<circle x="0" y="4.826" radius="2.54" width="0" layer="42"/>
<pad name="1" x="-2.54" y="-10.16" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-10.16" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-10.16" drill="1.1176" shape="long" rot="R90"/>
<text x="-5.461" y="-10.922" size="1.778" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="7.366" y="-11.049" size="1.778" layer="27" ratio="10" rot="R90">&gt;VALUE</text>
<rectangle x1="2.159" y1="-11.049" x2="2.921" y2="-10.414" layer="21"/>
<rectangle x1="-0.381" y1="-11.049" x2="0.381" y2="-10.414" layer="21"/>
<rectangle x1="-2.921" y1="-11.049" x2="-2.159" y2="-10.414" layer="21"/>
<rectangle x1="-3.175" y1="-10.414" x2="-1.905" y2="-7.62" layer="21"/>
<rectangle x1="-0.635" y1="-10.414" x2="0.635" y2="-7.62" layer="21"/>
<rectangle x1="1.905" y1="-10.414" x2="3.175" y2="-7.62" layer="21"/>
<hole x="0" y="4.826" drill="3.302"/>
</package>
<package name="TO39" urn="urn:adsk.eagle:footprint:16152/1" library_version="4">
<description>&lt;b&gt;Metal Can Package&lt;/b&gt;</description>
<wire x1="-4.0386" y1="-3.5306" x2="-3.5052" y2="-2.9972" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="-3.5306" x2="-3.5052" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-3.5052" y1="-4.064" x2="-4.0386" y2="-3.5306" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.8608" width="0.0508" layer="21"/>
<pad name="1" x="0" y="-2.54" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="0" y="2.54" drill="0.8128" shape="octagon"/>
<text x="-2.794" y="4.826" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.302" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO252" urn="urn:adsk.eagle:footprint:16153/1" library_version="4">
<description>&lt;b&gt;SMALL OUTLINE TRANSISTOR&lt;/b&gt;&lt;p&gt;
TS-003</description>
<wire x1="3.2766" y1="3.8354" x2="3.277" y2="-2.159" width="0.2032" layer="21"/>
<wire x1="3.277" y1="-2.159" x2="-3.277" y2="-2.159" width="0.2032" layer="21"/>
<wire x1="-3.277" y1="-2.159" x2="-3.2766" y2="3.8354" width="0.2032" layer="21"/>
<wire x1="-3.277" y1="3.835" x2="3.2774" y2="3.8346" width="0.2032" layer="51"/>
<wire x1="-2.5654" y1="3.937" x2="-2.5654" y2="4.6482" width="0.2032" layer="51"/>
<wire x1="-2.5654" y1="4.6482" x2="-2.1082" y2="5.1054" width="0.2032" layer="51"/>
<wire x1="-2.1082" y1="5.1054" x2="2.1082" y2="5.1054" width="0.2032" layer="51"/>
<wire x1="2.1082" y1="5.1054" x2="2.5654" y2="4.6482" width="0.2032" layer="51"/>
<wire x1="2.5654" y1="4.6482" x2="2.5654" y2="3.937" width="0.2032" layer="51"/>
<wire x1="2.5654" y1="3.937" x2="-2.5654" y2="3.937" width="0.2032" layer="51"/>
<smd name="3" x="0" y="2.5" dx="5.4" dy="6.2" layer="1"/>
<smd name="1" x="-2.28" y="-4.8" dx="1" dy="1.6" layer="1"/>
<smd name="2" x="2.28" y="-4.8" dx="1" dy="1.6" layer="1"/>
<text x="-3.81" y="-2.54" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="5.08" y="-2.54" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-2.7178" y1="-5.1562" x2="-1.8542" y2="-2.2606" layer="51"/>
<rectangle x1="1.8542" y1="-5.1562" x2="2.7178" y2="-2.2606" layer="51"/>
<rectangle x1="-0.4318" y1="-3.0226" x2="0.4318" y2="-2.2606" layer="21"/>
<polygon width="0.1998" layer="51">
<vertex x="-2.5654" y="3.937"/>
<vertex x="-2.5654" y="4.6482"/>
<vertex x="-2.1082" y="5.1054"/>
<vertex x="2.1082" y="5.1054"/>
<vertex x="2.5654" y="4.6482"/>
<vertex x="2.5654" y="3.937"/>
</polygon>
</package>
<package name="TO220V" urn="urn:adsk.eagle:footprint:16154/1" library_version="4">
<description>&lt;b&gt;TO 200 vertical&lt;/b&gt;</description>
<wire x1="5.08" y1="-1.143" x2="4.953" y2="-4.064" width="0.127" layer="21"/>
<wire x1="4.699" y1="-4.318" x2="4.953" y2="-4.064" width="0.127" layer="21"/>
<wire x1="4.699" y1="-4.318" x2="-4.699" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-4.064" x2="-4.699" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-4.064" x2="-5.08" y2="-1.143" width="0.127" layer="21"/>
<circle x="-4.4958" y="-3.7084" radius="0.254" width="0.127" layer="21"/>
<pad name="1" x="-2.54" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<text x="-5.08" y="-6.0452" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-7.62" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.175" y="-3.175" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="-3.175" size="1.27" layer="51" ratio="10">2</text>
<text x="1.905" y="-3.175" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="-5.334" y1="-0.762" x2="5.334" y2="0" layer="21"/>
<rectangle x1="-5.334" y1="-1.27" x2="-3.429" y2="-0.762" layer="21"/>
<rectangle x1="-1.651" y1="-1.27" x2="-0.889" y2="-0.762" layer="21"/>
<rectangle x1="-3.429" y1="-1.27" x2="-1.651" y2="-0.762" layer="51"/>
<rectangle x1="0.889" y1="-1.27" x2="1.651" y2="-0.762" layer="21"/>
<rectangle x1="3.429" y1="-1.27" x2="5.334" y2="-0.762" layer="21"/>
<rectangle x1="-0.889" y1="-1.27" x2="0.889" y2="-0.762" layer="51"/>
<rectangle x1="1.651" y1="-1.27" x2="3.429" y2="-0.762" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="TO92" urn="urn:adsk.eagle:package:16416/2" type="model" library_version="4">
<description>TO-92</description>
<packageinstances>
<packageinstance name="TO92"/>
</packageinstances>
</package3d>
<package3d name="TO220H" urn="urn:adsk.eagle:package:16414/1" type="box" library_version="4">
<description>TO-220</description>
<packageinstances>
<packageinstance name="TO220H"/>
</packageinstances>
</package3d>
<package3d name="TO39" urn="urn:adsk.eagle:package:16419/1" type="box" library_version="4">
<description>Metal Can Package</description>
<packageinstances>
<packageinstance name="TO39"/>
</packageinstances>
</package3d>
<package3d name="TO252" urn="urn:adsk.eagle:package:16415/2" type="model" library_version="4">
<description>SMALL OUTLINE TRANSISTOR
TS-003</description>
<packageinstances>
<packageinstance name="TO252"/>
</packageinstances>
</package3d>
<package3d name="TO220V" urn="urn:adsk.eagle:package:16417/2" type="model" library_version="4">
<description>TO 200 vertical</description>
<packageinstances>
<packageinstance name="TO220V"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="78XX" urn="urn:adsk.eagle:symbol:16149/1" library_version="4">
<wire x1="-7.62" y1="-5.08" x2="7.62" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="7.62" y2="2.54" width="0.4064" layer="94"/>
<wire x1="7.62" y1="2.54" x2="-7.62" y2="2.54" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="2.54" x2="-7.62" y2="-5.08" width="0.4064" layer="94"/>
<text x="-7.62" y="5.715" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="3.175" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.032" y="-4.318" size="1.524" layer="95">GND</text>
<pin name="VI" x="-10.16" y="0" length="short" direction="in"/>
<pin name="GND" x="0" y="-7.62" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="VO" x="10.16" y="0" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="78*" urn="urn:adsk.eagle:component:16686/4" prefix="IC" library_version="4">
<description>Positive &lt;b&gt;VOLTAGE REGULATOR&lt;/b&gt;&lt;p&gt;
Source:&lt;br&gt;
http://cache.national.com/ds/LM/LM78L05.pdf&lt;br&gt;
http://www.fairchildsemi.com/ds/LM/LM7805.pdf</description>
<gates>
<gate name="A1" symbol="78XX" x="0" y="0"/>
</gates>
<devices>
<device name="Z" package="TO92">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="3"/>
<connect gate="A1" pin="VO" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16416/2"/>
</package3dinstances>
<technologies>
<technology name="L05"/>
<technology name="L08"/>
<technology name="L12"/>
<technology name="L15"/>
<technology name="L18"/>
<technology name="L24"/>
</technologies>
</device>
<device name="T" package="TO220H">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16414/1"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="H" package="TO39">
<connects>
<connect gate="A1" pin="GND" pad="3"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16419/1"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="L" package="TO92">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="3"/>
<connect gate="A1" pin="VO" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16416/2"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="DT" package="TO252">
<connects>
<connect gate="A1" pin="GND" pad="3"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16415/2"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="TV" package="TO220V">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16417/2"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
<technology name="6"/>
<technology name="8"/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-lstb" urn="urn:adsk.eagle:library:162">
<description>&lt;b&gt;Pin Headers&lt;/b&gt;&lt;p&gt;
Naming:&lt;p&gt;
MA = male&lt;p&gt;
# contacts - # rows&lt;p&gt;
W = angled&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="MA08-1" urn="urn:adsk.eagle:footprint:8294/1" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-9.525" y1="1.27" x2="-8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.27" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-0.635" x2="-8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-1.27" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-1.27" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="1.27" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-0.635" x2="-9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-1.27" x2="-9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="-1.27" x2="-5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-1.27" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="-1.27" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="-1.27" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<pad name="1" x="-8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-10.16" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-9.398" y="-2.921" size="1.27" layer="21" ratio="10">1</text>
<text x="8.255" y="1.651" size="1.27" layer="21" ratio="10">8</text>
<text x="-1.27" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-6.604" y1="-0.254" x2="-6.096" y2="0.254" layer="51"/>
<rectangle x1="-9.144" y1="-0.254" x2="-8.636" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="6.096" y1="-0.254" x2="6.604" y2="0.254" layer="51"/>
<rectangle x1="8.636" y1="-0.254" x2="9.144" y2="0.254" layer="51"/>
</package>
<package name="MA10-1" urn="urn:adsk.eagle:footprint:8300/1" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-12.065" y1="1.27" x2="-10.795" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="1.27" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-0.635" x2="-10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="1.27" x2="-8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.27" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-0.635" x2="-8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-1.27" x2="-9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="-1.27" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="0.635" x2="-12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="1.27" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-0.635" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="-1.27" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-1.27" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-1.27" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-1.27" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="10.795" y2="1.27" width="0.1524" layer="21"/>
<wire x1="10.795" y1="1.27" x2="12.065" y2="1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="1.27" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-0.635" x2="12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="-1.27" x2="10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="10.795" y1="-1.27" x2="10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="-1.27" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="12.7" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-12.7" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-11.938" y="-2.921" size="1.27" layer="21" ratio="10">1</text>
<text x="10.795" y="1.651" size="1.27" layer="21" ratio="10">10</text>
<text x="1.27" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-9.144" y1="-0.254" x2="-8.636" y2="0.254" layer="51"/>
<rectangle x1="-11.684" y1="-0.254" x2="-11.176" y2="0.254" layer="51"/>
<rectangle x1="-6.604" y1="-0.254" x2="-6.096" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
<rectangle x1="6.096" y1="-0.254" x2="6.604" y2="0.254" layer="51"/>
<rectangle x1="11.176" y1="-0.254" x2="11.684" y2="0.254" layer="51"/>
<rectangle x1="8.636" y1="-0.254" x2="9.144" y2="0.254" layer="51"/>
</package>
<package name="MA18-2" urn="urn:adsk.eagle:footprint:8317/1" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-22.225" y1="2.54" x2="-20.955" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-20.955" y1="2.54" x2="-20.32" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-20.32" y1="1.905" x2="-19.685" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-19.685" y1="2.54" x2="-18.415" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-18.415" y1="2.54" x2="-17.78" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-22.225" y1="2.54" x2="-22.86" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="1.905" x2="-17.145" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-17.145" y1="2.54" x2="-15.875" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-15.875" y1="2.54" x2="-15.24" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-14.605" y1="2.54" x2="-13.335" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="2.54" x2="-12.7" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.065" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="2.54" x2="-10.795" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="2.54" x2="-10.16" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-14.605" y1="2.54" x2="-15.24" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="1.905" x2="-9.525" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="2.54" x2="-8.255" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="2.54" x2="-7.62" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-20.32" y1="-1.905" x2="-20.955" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-17.78" y1="-1.905" x2="-18.415" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-18.415" y1="-2.54" x2="-19.685" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-19.685" y1="-2.54" x2="-20.32" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-22.86" y1="1.905" x2="-22.86" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-22.86" y1="-1.905" x2="-22.225" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-20.955" y1="-2.54" x2="-22.225" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="-1.905" x2="-15.875" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-15.875" y1="-2.54" x2="-17.145" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-17.145" y1="-2.54" x2="-17.78" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-1.905" x2="-13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-1.905" x2="-10.795" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="-2.54" x2="-12.065" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="-2.54" x2="-12.7" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-15.24" y1="-1.905" x2="-14.605" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-2.54" x2="-14.605" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-1.905" x2="-8.255" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.54" x2="-9.525" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="-2.54" x2="-10.16" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="2.54" x2="-5.08" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="2.54" x2="-5.715" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-6.985" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-2.54" x2="-7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-2.54" x2="-6.985" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-5.715" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="2.54" x2="-3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.905" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="2.54" x2="-5.08" y2="1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="1.905" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="4.445" y2="2.54" width="0.1524" layer="21"/>
<wire x1="4.445" y1="2.54" x2="5.08" y2="1.905" width="0.1524" layer="21"/>
<wire x1="5.08" y1="1.905" x2="5.715" y2="2.54" width="0.1524" layer="21"/>
<wire x1="5.715" y1="2.54" x2="6.985" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.985" y1="2.54" x2="7.62" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="7.62" y1="1.905" x2="8.255" y2="2.54" width="0.1524" layer="21"/>
<wire x1="8.255" y1="2.54" x2="9.525" y2="2.54" width="0.1524" layer="21"/>
<wire x1="9.525" y1="2.54" x2="10.16" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-2.54" x2="-4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="4.445" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-1.905" x2="6.985" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-2.54" x2="5.715" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.715" y1="-2.54" x2="5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-2.54" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-1.905" x2="9.525" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="9.525" y1="-2.54" x2="8.255" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.54" x2="7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="12.065" y1="2.54" x2="12.7" y2="1.905" width="0.1524" layer="21"/>
<wire x1="10.795" y1="2.54" x2="12.065" y2="2.54" width="0.1524" layer="21"/>
<wire x1="10.16" y1="1.905" x2="10.795" y2="2.54" width="0.1524" layer="21"/>
<wire x1="10.795" y1="-2.54" x2="10.16" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="12.065" y1="-2.54" x2="10.795" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-1.905" x2="12.065" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="13.335" y1="2.54" x2="14.605" y2="2.54" width="0.1524" layer="21"/>
<wire x1="14.605" y1="2.54" x2="15.24" y2="1.905" width="0.1524" layer="21"/>
<wire x1="15.24" y1="1.905" x2="15.875" y2="2.54" width="0.1524" layer="21"/>
<wire x1="15.875" y1="2.54" x2="17.145" y2="2.54" width="0.1524" layer="21"/>
<wire x1="17.145" y1="2.54" x2="17.78" y2="1.905" width="0.1524" layer="21"/>
<wire x1="13.335" y1="2.54" x2="12.7" y2="1.905" width="0.1524" layer="21"/>
<wire x1="17.78" y1="1.905" x2="18.415" y2="2.54" width="0.1524" layer="21"/>
<wire x1="18.415" y1="2.54" x2="19.685" y2="2.54" width="0.1524" layer="21"/>
<wire x1="19.685" y1="2.54" x2="20.32" y2="1.905" width="0.1524" layer="21"/>
<wire x1="20.32" y1="1.905" x2="20.955" y2="2.54" width="0.1524" layer="21"/>
<wire x1="20.955" y1="2.54" x2="22.225" y2="2.54" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-1.905" x2="14.605" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="17.78" y1="-1.905" x2="17.145" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="17.145" y1="-2.54" x2="15.875" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="15.875" y1="-2.54" x2="15.24" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-1.905" x2="13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="14.605" y1="-2.54" x2="13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="22.225" y1="-2.54" x2="20.955" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="20.955" y1="-2.54" x2="20.32" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-1.905" x2="19.685" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="19.685" y1="-2.54" x2="18.415" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="18.415" y1="-2.54" x2="17.78" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="22.86" y1="1.905" x2="22.86" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="22.225" y1="2.54" x2="22.86" y2="1.905" width="0.1524" layer="21"/>
<wire x1="22.86" y1="-1.905" x2="22.225" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-21.59" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="3" x="-19.05" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="5" x="-16.51" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="7" x="-13.97" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="9" x="-11.43" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="11" x="-8.89" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="2" x="-21.59" y="1.27" drill="1.016" shape="octagon"/>
<pad name="4" x="-19.05" y="1.27" drill="1.016" shape="octagon"/>
<pad name="6" x="-16.51" y="1.27" drill="1.016" shape="octagon"/>
<pad name="8" x="-13.97" y="1.27" drill="1.016" shape="octagon"/>
<pad name="10" x="-11.43" y="1.27" drill="1.016" shape="octagon"/>
<pad name="12" x="-8.89" y="1.27" drill="1.016" shape="octagon"/>
<pad name="13" x="-6.35" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="14" x="-6.35" y="1.27" drill="1.016" shape="octagon"/>
<pad name="15" x="-3.81" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="17" x="-1.27" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="19" x="1.27" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="21" x="3.81" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="23" x="6.35" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="25" x="8.89" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="16" x="-3.81" y="1.27" drill="1.016" shape="octagon"/>
<pad name="18" x="-1.27" y="1.27" drill="1.016" shape="octagon"/>
<pad name="20" x="1.27" y="1.27" drill="1.016" shape="octagon"/>
<pad name="22" x="3.81" y="1.27" drill="1.016" shape="octagon"/>
<pad name="24" x="6.35" y="1.27" drill="1.016" shape="octagon"/>
<pad name="26" x="8.89" y="1.27" drill="1.016" shape="octagon"/>
<pad name="27" x="11.43" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="28" x="11.43" y="1.27" drill="1.016" shape="octagon"/>
<pad name="29" x="13.97" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="31" x="16.51" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="33" x="19.05" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="30" x="13.97" y="1.27" drill="1.016" shape="octagon"/>
<pad name="32" x="16.51" y="1.27" drill="1.016" shape="octagon"/>
<pad name="34" x="19.05" y="1.27" drill="1.016" shape="octagon"/>
<pad name="35" x="21.59" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="36" x="21.59" y="1.27" drill="1.016" shape="octagon"/>
<text x="-22.098" y="-4.191" size="1.27" layer="21" ratio="10">1</text>
<text x="-22.86" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="7.62" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="20.32" y="2.921" size="1.27" layer="21" ratio="10">36</text>
<rectangle x1="-19.304" y1="-1.524" x2="-18.796" y2="-1.016" layer="51"/>
<rectangle x1="-21.844" y1="-1.524" x2="-21.336" y2="-1.016" layer="51"/>
<rectangle x1="-16.764" y1="-1.524" x2="-16.256" y2="-1.016" layer="51"/>
<rectangle x1="-11.684" y1="-1.524" x2="-11.176" y2="-1.016" layer="51"/>
<rectangle x1="-14.224" y1="-1.524" x2="-13.716" y2="-1.016" layer="51"/>
<rectangle x1="-9.144" y1="-1.524" x2="-8.636" y2="-1.016" layer="51"/>
<rectangle x1="-21.844" y1="1.016" x2="-21.336" y2="1.524" layer="51"/>
<rectangle x1="-19.304" y1="1.016" x2="-18.796" y2="1.524" layer="51"/>
<rectangle x1="-16.764" y1="1.016" x2="-16.256" y2="1.524" layer="51"/>
<rectangle x1="-14.224" y1="1.016" x2="-13.716" y2="1.524" layer="51"/>
<rectangle x1="-11.684" y1="1.016" x2="-11.176" y2="1.524" layer="51"/>
<rectangle x1="-9.144" y1="1.016" x2="-8.636" y2="1.524" layer="51"/>
<rectangle x1="-6.604" y1="1.016" x2="-6.096" y2="1.524" layer="51"/>
<rectangle x1="-6.604" y1="-1.524" x2="-6.096" y2="-1.016" layer="51"/>
<rectangle x1="-1.524" y1="-1.524" x2="-1.016" y2="-1.016" layer="51"/>
<rectangle x1="-4.064" y1="-1.524" x2="-3.556" y2="-1.016" layer="51"/>
<rectangle x1="1.016" y1="-1.524" x2="1.524" y2="-1.016" layer="51"/>
<rectangle x1="6.096" y1="-1.524" x2="6.604" y2="-1.016" layer="51"/>
<rectangle x1="3.556" y1="-1.524" x2="4.064" y2="-1.016" layer="51"/>
<rectangle x1="8.636" y1="-1.524" x2="9.144" y2="-1.016" layer="51"/>
<rectangle x1="-4.064" y1="1.016" x2="-3.556" y2="1.524" layer="51"/>
<rectangle x1="-1.524" y1="1.016" x2="-1.016" y2="1.524" layer="51"/>
<rectangle x1="1.016" y1="1.016" x2="1.524" y2="1.524" layer="51"/>
<rectangle x1="3.556" y1="1.016" x2="4.064" y2="1.524" layer="51"/>
<rectangle x1="6.096" y1="1.016" x2="6.604" y2="1.524" layer="51"/>
<rectangle x1="8.636" y1="1.016" x2="9.144" y2="1.524" layer="51"/>
<rectangle x1="11.176" y1="1.016" x2="11.684" y2="1.524" layer="51"/>
<rectangle x1="11.176" y1="-1.524" x2="11.684" y2="-1.016" layer="51"/>
<rectangle x1="16.256" y1="-1.524" x2="16.764" y2="-1.016" layer="51"/>
<rectangle x1="13.716" y1="-1.524" x2="14.224" y2="-1.016" layer="51"/>
<rectangle x1="18.796" y1="-1.524" x2="19.304" y2="-1.016" layer="51"/>
<rectangle x1="13.716" y1="1.016" x2="14.224" y2="1.524" layer="51"/>
<rectangle x1="16.256" y1="1.016" x2="16.764" y2="1.524" layer="51"/>
<rectangle x1="18.796" y1="1.016" x2="19.304" y2="1.524" layer="51"/>
<rectangle x1="21.336" y1="-1.524" x2="21.844" y2="-1.016" layer="51"/>
<rectangle x1="21.336" y1="1.016" x2="21.844" y2="1.524" layer="51"/>
</package>
<package name="MA03-1" urn="urn:adsk.eagle:footprint:8281/1" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="1.27" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="0.635" x2="-3.81" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="0.635" x2="3.81" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-3.81" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-0.635" size="1.27" layer="21" ratio="10">1</text>
<text x="-3.81" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
<rectangle x1="-2.794" y1="-0.254" x2="-2.286" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="MA08-1" urn="urn:adsk.eagle:package:8343/1" type="box" library_version="1">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA08-1"/>
</packageinstances>
</package3d>
<package3d name="MA10-1" urn="urn:adsk.eagle:package:8346/1" type="box" library_version="1">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA10-1"/>
</packageinstances>
</package3d>
<package3d name="MA18-2" urn="urn:adsk.eagle:package:8360/1" type="box" library_version="1">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA18-2"/>
</packageinstances>
</package3d>
<package3d name="MA03-1" urn="urn:adsk.eagle:package:8339/1" type="box" library_version="1">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA03-1"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="MA08-1" urn="urn:adsk.eagle:symbol:8293/1" library_version="1">
<wire x1="3.81" y1="-10.16" x2="-1.27" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-7.62" x2="2.54" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="10.16" x2="2.54" y2="10.16" width="0.6096" layer="94"/>
<wire x1="1.27" y1="7.62" x2="2.54" y2="7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="5.08" x2="2.54" y2="5.08" width="0.6096" layer="94"/>
<wire x1="-1.27" y1="12.7" x2="-1.27" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-10.16" x2="3.81" y2="12.7" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="12.7" x2="3.81" y2="12.7" width="0.4064" layer="94"/>
<text x="-1.27" y="-12.7" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="13.462" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="6" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="7" x="7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="8" x="7.62" y="10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="MA10-1" urn="urn:adsk.eagle:symbol:8299/1" library_version="1">
<wire x1="3.81" y1="-12.7" x2="-1.27" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-7.62" x2="2.54" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-10.16" x2="2.54" y2="-10.16" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="7.62" x2="2.54" y2="7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="5.08" x2="2.54" y2="5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="12.7" x2="2.54" y2="12.7" width="0.6096" layer="94"/>
<wire x1="1.27" y1="10.16" x2="2.54" y2="10.16" width="0.6096" layer="94"/>
<wire x1="-1.27" y1="15.24" x2="-1.27" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-12.7" x2="3.81" y2="15.24" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="15.24" x2="3.81" y2="15.24" width="0.4064" layer="94"/>
<text x="-1.27" y="-15.24" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="16.002" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="-7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="6" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="7" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="8" x="7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="9" x="7.62" y="10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="10" x="7.62" y="12.7" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="MA18-2" urn="urn:adsk.eagle:symbol:8316/1" library_version="1">
<wire x1="3.81" y1="-22.86" x2="-3.81" y2="-22.86" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-15.24" x2="2.54" y2="-15.24" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-17.78" x2="2.54" y2="-17.78" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-20.32" x2="2.54" y2="-20.32" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-15.24" x2="-1.27" y2="-15.24" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-17.78" x2="-1.27" y2="-17.78" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-20.32" x2="-1.27" y2="-20.32" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-10.16" x2="2.54" y2="-10.16" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-12.7" x2="2.54" y2="-12.7" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-10.16" x2="-1.27" y2="-10.16" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-12.7" x2="-1.27" y2="-12.7" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-7.62" x2="2.54" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-1.27" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-1.27" y2="-7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="10.16" x2="2.54" y2="10.16" width="0.6096" layer="94"/>
<wire x1="1.27" y1="7.62" x2="2.54" y2="7.62" width="0.6096" layer="94"/>
<wire x1="1.27" y1="5.08" x2="2.54" y2="5.08" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="10.16" x2="-1.27" y2="10.16" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="7.62" x2="-1.27" y2="7.62" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-1.27" y2="5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="15.24" x2="2.54" y2="15.24" width="0.6096" layer="94"/>
<wire x1="1.27" y1="12.7" x2="2.54" y2="12.7" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="15.24" x2="-1.27" y2="15.24" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="12.7" x2="-1.27" y2="12.7" width="0.6096" layer="94"/>
<wire x1="1.27" y1="20.32" x2="2.54" y2="20.32" width="0.6096" layer="94"/>
<wire x1="1.27" y1="17.78" x2="2.54" y2="17.78" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="20.32" x2="-1.27" y2="20.32" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="17.78" x2="-1.27" y2="17.78" width="0.6096" layer="94"/>
<wire x1="-3.81" y1="25.4" x2="-3.81" y2="-22.86" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-22.86" x2="3.81" y2="25.4" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="25.4" x2="3.81" y2="25.4" width="0.4064" layer="94"/>
<wire x1="1.27" y1="22.86" x2="2.54" y2="22.86" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="22.86" x2="-1.27" y2="22.86" width="0.6096" layer="94"/>
<text x="-3.81" y="-25.4" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="26.162" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-20.32" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="-17.78" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="-15.24" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-20.32" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="4" x="-7.62" y="-17.78" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="6" x="-7.62" y="-15.24" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="7" x="7.62" y="-12.7" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="9" x="7.62" y="-10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="8" x="-7.62" y="-12.7" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="10" x="-7.62" y="-10.16" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="11" x="7.62" y="-7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="13" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="15" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="12" x="-7.62" y="-7.62" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="14" x="-7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="16" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="17" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="19" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="18" x="-7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="20" x="-7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="21" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="23" x="7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="25" x="7.62" y="10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="22" x="-7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="24" x="-7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="26" x="-7.62" y="10.16" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="27" x="7.62" y="12.7" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="29" x="7.62" y="15.24" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="28" x="-7.62" y="12.7" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="30" x="-7.62" y="15.24" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="31" x="7.62" y="17.78" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="33" x="7.62" y="20.32" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="32" x="-7.62" y="17.78" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="34" x="-7.62" y="20.32" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="35" x="7.62" y="22.86" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="36" x="-7.62" y="22.86" visible="pad" length="middle" direction="pas" swaplevel="1"/>
</symbol>
<symbol name="MA03-1" urn="urn:adsk.eagle:symbol:8280/1" library_version="1">
<wire x1="3.81" y1="-5.08" x2="-1.27" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-1.27" y1="5.08" x2="-1.27" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<text x="-1.27" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MA08-1" urn="urn:adsk.eagle:component:8385/1" prefix="SV" uservalue="yes" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA08-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA08-1">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
<connect gate="1" pin="7" pad="7"/>
<connect gate="1" pin="8" pad="8"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8343/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MA10-1" urn="urn:adsk.eagle:component:8394/1" prefix="SV" uservalue="yes" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA10-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA10-1">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="10" pad="10"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
<connect gate="1" pin="7" pad="7"/>
<connect gate="1" pin="8" pad="8"/>
<connect gate="1" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8346/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MA18-2" urn="urn:adsk.eagle:component:8399/1" prefix="SV" uservalue="yes" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="MA18-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA18-2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="11" pad="11"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="13" pad="13"/>
<connect gate="G$1" pin="14" pad="14"/>
<connect gate="G$1" pin="15" pad="15"/>
<connect gate="G$1" pin="16" pad="16"/>
<connect gate="G$1" pin="17" pad="17"/>
<connect gate="G$1" pin="18" pad="18"/>
<connect gate="G$1" pin="19" pad="19"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="20" pad="20"/>
<connect gate="G$1" pin="21" pad="21"/>
<connect gate="G$1" pin="22" pad="22"/>
<connect gate="G$1" pin="23" pad="23"/>
<connect gate="G$1" pin="24" pad="24"/>
<connect gate="G$1" pin="25" pad="25"/>
<connect gate="G$1" pin="26" pad="26"/>
<connect gate="G$1" pin="27" pad="27"/>
<connect gate="G$1" pin="28" pad="28"/>
<connect gate="G$1" pin="29" pad="29"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="30" pad="30"/>
<connect gate="G$1" pin="31" pad="31"/>
<connect gate="G$1" pin="32" pad="32"/>
<connect gate="G$1" pin="33" pad="33"/>
<connect gate="G$1" pin="34" pad="34"/>
<connect gate="G$1" pin="35" pad="35"/>
<connect gate="G$1" pin="36" pad="36"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8360/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MA03-1" urn="urn:adsk.eagle:component:8376/1" prefix="SV" uservalue="yes" library_version="1">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="MA03-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA03-1">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8339/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1" urn="urn:adsk.eagle:library:371">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26925/1" library_version="1">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+3V3" urn="urn:adsk.eagle:symbol:26950/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+3V3" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="+5V" urn="urn:adsk.eagle:symbol:26929/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="+18V" urn="urn:adsk.eagle:symbol:26933/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<wire x1="1.27" y1="-0.635" x2="0" y2="1.27" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="-1.27" y2="-0.635" width="0.254" layer="94"/>
<wire x1="0" y1="2.54" x2="-1.27" y2="0.635" width="0.254" layer="94"/>
<wire x1="1.27" y1="0.635" x2="0" y2="2.54" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+18V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:26954/1" prefix="GND" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+3V3" urn="urn:adsk.eagle:component:26981/1" prefix="+3V3" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="+3V3" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+5V" urn="urn:adsk.eagle:component:26963/1" prefix="P+" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+5V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+18V" urn="urn:adsk.eagle:component:26960/1" prefix="P+" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+18V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="ultrasonic">
<packages>
<package name="TCT40-16">
<pad name="P1" x="5" y="0" drill="1" diameter="3"/>
<pad name="P0" x="-5" y="0" drill="1" diameter="3"/>
<circle x="0" y="0" radius="8" width="0.4064" layer="51"/>
<text x="-3" y="2" size="1.27" layer="25">&gt;NAME</text>
<text x="-3" y="-4" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="TCT40-16">
<pin name="P0" x="5.08" y="2.54" length="short" rot="R180"/>
<pin name="P1" x="5.08" y="0" length="short" rot="R180"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="3.81" width="0.254" layer="94"/>
<wire x1="1.27" y1="3.81" x2="2.54" y2="3.81" width="0.254" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="-1.27" width="0.254" layer="94"/>
<wire x1="2.54" y1="-1.27" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="3.81" x2="-2.54" y2="7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="7.62" x2="-2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<text x="1.27" y="-2.54" size="1.27" layer="95">&gt;NAME</text>
<text x="1.27" y="-5.08" size="1.27" layer="96">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="TCT40-16" uservalue="yes">
<description>TCT40-16 T/R Ultrasonic transducer.
40kHz.
Diameter: 16mm</description>
<gates>
<gate name="G$1" symbol="TCT40-16" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TCT40-16">
<connects>
<connect gate="G$1" pin="P0" pad="P0"/>
<connect gate="G$1" pin="P1" pad="P1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="switch-omron" urn="urn:adsk.eagle:library:377">
<description>&lt;b&gt;Omron Switches&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="B3F-10XX" urn="urn:adsk.eagle:footprint:27476/1" library_version="2">
<description>&lt;b&gt;OMRON SWITCH&lt;/b&gt;</description>
<wire x1="3.302" y1="-0.762" x2="3.048" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-0.762" x2="3.302" y2="0.762" width="0.1524" layer="21"/>
<wire x1="3.048" y1="0.762" x2="3.302" y2="0.762" width="0.1524" layer="21"/>
<wire x1="3.048" y1="1.016" x2="3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.302" y1="0.762" x2="-3.048" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="0.762" x2="-3.302" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-3.048" y1="-0.762" x2="-3.302" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="3.048" y1="2.54" x2="2.54" y2="3.048" width="0.1524" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-3.048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-3.048" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="-1.27" width="0.0508" layer="51"/>
<wire x1="1.27" y1="-1.27" x2="-1.27" y2="-1.27" width="0.0508" layer="51"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="1.27" width="0.0508" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="1.27" y2="1.27" width="0.0508" layer="51"/>
<wire x1="-1.27" y1="3.048" x2="-1.27" y2="2.794" width="0.0508" layer="21"/>
<wire x1="1.27" y1="2.794" x2="-1.27" y2="2.794" width="0.0508" layer="21"/>
<wire x1="1.27" y1="2.794" x2="1.27" y2="3.048" width="0.0508" layer="21"/>
<wire x1="1.143" y1="-2.794" x2="-1.27" y2="-2.794" width="0.0508" layer="21"/>
<wire x1="1.143" y1="-2.794" x2="1.143" y2="-3.048" width="0.0508" layer="21"/>
<wire x1="-1.27" y1="-2.794" x2="-1.27" y2="-3.048" width="0.0508" layer="21"/>
<wire x1="2.54" y1="-3.048" x2="2.159" y2="-3.048" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-2.159" y2="-3.048" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="-3.048" x2="-1.27" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="3.048" x2="-2.159" y2="3.048" width="0.1524" layer="51"/>
<wire x1="2.54" y1="3.048" x2="2.159" y2="3.048" width="0.1524" layer="51"/>
<wire x1="2.159" y1="3.048" x2="1.27" y2="3.048" width="0.1524" layer="21"/>
<wire x1="1.27" y1="3.048" x2="-1.27" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="3.048" x2="-2.159" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-3.048" x2="1.143" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-3.048" x2="2.159" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="3.048" y1="-0.762" x2="3.048" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="3.048" y1="0.762" x2="3.048" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-3.048" y1="-0.762" x2="-3.048" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.048" y1="0.762" x2="-3.048" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-2.159" x2="1.27" y2="-2.159" width="0.1524" layer="51"/>
<wire x1="1.27" y1="2.286" x2="-1.27" y2="2.286" width="0.1524" layer="51"/>
<wire x1="-2.413" y1="1.27" x2="-2.413" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.413" y1="-0.508" x2="-2.413" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-2.413" y1="0.508" x2="-2.159" y2="-0.381" width="0.1524" layer="51"/>
<circle x="0" y="0" radius="1.778" width="0.1524" layer="21"/>
<circle x="-2.159" y="-2.159" radius="0.508" width="0.1524" layer="51"/>
<circle x="2.159" y="-2.032" radius="0.508" width="0.1524" layer="51"/>
<circle x="2.159" y="2.159" radius="0.508" width="0.1524" layer="51"/>
<circle x="-2.159" y="2.159" radius="0.508" width="0.1524" layer="51"/>
<circle x="0" y="0" radius="0.635" width="0.0508" layer="51"/>
<circle x="0" y="0" radius="0.254" width="0.1524" layer="21"/>
<pad name="1" x="-3.2512" y="2.2606" drill="1.016" shape="long"/>
<pad name="3" x="-3.2512" y="-2.2606" drill="1.016" shape="long"/>
<pad name="2" x="3.2512" y="2.2606" drill="1.016" shape="long"/>
<pad name="4" x="3.2512" y="-2.2606" drill="1.016" shape="long"/>
<text x="-3.048" y="3.683" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.048" y="-5.08" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.318" y="1.651" size="1.27" layer="51" ratio="10">1</text>
<text x="3.556" y="1.524" size="1.27" layer="51" ratio="10">2</text>
<text x="-4.572" y="-2.794" size="1.27" layer="51" ratio="10">3</text>
<text x="3.556" y="-2.794" size="1.27" layer="51" ratio="10">4</text>
</package>
</packages>
<packages3d>
<package3d name="B3F-10XX" urn="urn:adsk.eagle:package:27496/1" type="box" library_version="2">
<description>OMRON SWITCH</description>
<packageinstances>
<packageinstance name="B3F-10XX"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="TS2" urn="urn:adsk.eagle:symbol:27469/1" library_version="2">
<wire x1="0" y1="1.905" x2="0" y2="2.54" width="0.254" layer="94"/>
<wire x1="-4.445" y1="1.905" x2="-3.175" y2="1.905" width="0.254" layer="94"/>
<wire x1="-4.445" y1="-1.905" x2="-3.175" y2="-1.905" width="0.254" layer="94"/>
<wire x1="-4.445" y1="1.905" x2="-4.445" y2="0" width="0.254" layer="94"/>
<wire x1="-4.445" y1="0" x2="-4.445" y2="-1.905" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.905" y2="0" width="0.1524" layer="94"/>
<wire x1="-1.27" y1="0" x2="-0.635" y2="0" width="0.1524" layer="94"/>
<wire x1="-4.445" y1="0" x2="-3.175" y2="0" width="0.1524" layer="94"/>
<wire x1="2.54" y1="2.54" x2="0" y2="2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="1.905" width="0.254" layer="94"/>
<circle x="0" y="-2.54" radius="0.127" width="0.4064" layer="94"/>
<circle x="0" y="2.54" radius="0.127" width="0.4064" layer="94"/>
<text x="-6.35" y="-2.54" size="1.778" layer="95" rot="R90">&gt;NAME</text>
<text x="-3.81" y="3.175" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="P" x="0" y="-5.08" visible="pad" length="short" direction="pas" swaplevel="2" rot="R90"/>
<pin name="S" x="0" y="5.08" visible="pad" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="S1" x="2.54" y="5.08" visible="pad" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="P1" x="2.54" y="-5.08" visible="pad" length="short" direction="pas" swaplevel="2" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="10-XX" urn="urn:adsk.eagle:component:27498/1" prefix="S" uservalue="yes" library_version="2">
<description>&lt;b&gt;OMRON SWITCH&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="TS2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B3F-10XX">
<connects>
<connect gate="1" pin="P" pad="3"/>
<connect gate="1" pin="P1" pad="4"/>
<connect gate="1" pin="S" pad="1"/>
<connect gate="1" pin="S1" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:27496/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="B3F-1000" constant="no"/>
<attribute name="OC_FARNELL" value="176432" constant="no"/>
<attribute name="OC_NEWARK" value="36M3542" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="TCT40">
<packages>
<package name="TCT40-16">
<pad name="P1" x="5" y="0" drill="1" diameter="3"/>
<pad name="P0" x="-5" y="0" drill="1" diameter="3"/>
<circle x="0" y="0" radius="8" width="0.4064" layer="51"/>
<text x="-3" y="2" size="1.27" layer="25">&gt;NAME</text>
<text x="-3" y="-4" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="TCT40-16">
<pin name="P0" x="5.08" y="2.54" length="short" rot="R180"/>
<pin name="P1" x="5.08" y="0" length="short" rot="R180"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="3.81" width="0.254" layer="94"/>
<wire x1="1.27" y1="3.81" x2="2.54" y2="3.81" width="0.254" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="-1.27" width="0.254" layer="94"/>
<wire x1="2.54" y1="-1.27" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="3.81" x2="-2.54" y2="7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="7.62" x2="-2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<text x="1.27" y="-2.54" size="1.27" layer="95">&gt;NAME</text>
<text x="1.27" y="-5.08" size="1.27" layer="96">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="TCT40-16" uservalue="yes">
<description>TCT40-16 T/R Ultrasonic transducer.
40kHz.
Diameter: 16mm</description>
<gates>
<gate name="G$1" symbol="TCT40-16" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TCT40-16">
<connects>
<connect gate="G$1" pin="P0" pad="P0"/>
<connect gate="G$1" pin="P1" pad="P1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="DCJack">
<packages>
<package name="DCJACK">
<wire x1="-4.5" y1="-3" x2="4.5" y2="-3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="-3" x2="-4.5" y2="11" width="0.1524" layer="21"/>
<wire x1="4.5" y1="-3" x2="4.5" y2="11" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="11" x2="-3" y2="11" width="0.1524" layer="21"/>
<wire x1="-3" y1="11" x2="3" y2="11" width="0.1524" layer="21"/>
<wire x1="3" y1="11" x2="4.5" y2="11" width="0.1524" layer="21"/>
<wire x1="-3" y1="11" x2="-2.9" y2="0.1" width="0.1524" layer="21" style="shortdash"/>
<wire x1="-2.9" y1="0.1" x2="-0.9" y2="0.1" width="0.1524" layer="21" style="shortdash"/>
<wire x1="-0.9" y1="0.1" x2="1.1" y2="0.1" width="0.1524" layer="21" style="shortdash"/>
<wire x1="1.1" y1="0.1" x2="3.1" y2="0.1" width="0.1524" layer="21" style="shortdash"/>
<wire x1="3.1" y1="0.1" x2="3" y2="11" width="0.1524" layer="21" style="shortdash"/>
<wire x1="1.1" y1="0.1" x2="1" y2="8.5" width="0.1524" layer="21" style="shortdash"/>
<wire x1="-0.9" y1="0.1" x2="-1" y2="8.5" width="0.1524" layer="21" style="shortdash"/>
<wire x1="-1" y1="8.5" x2="1" y2="8.5" width="0.1524" layer="21" curve="-180"/>
<text x="-4.7" y="2.6" size="1.778" layer="25" rot="R90">&gt;NAME</text>
<pad name="1" x="0" y="-3" drill="1" diameter="4.5" shape="octagon"/>
<pad name="2" x="0" y="3" drill="1" diameter="4.5" shape="octagon"/>
<pad name="3" x="-5" y="0" drill="1" diameter="4.5" shape="octagon" rot="R90"/>
</package>
</packages>
<symbols>
<symbol name="DCJACK">
<pin name="GND" x="-1.27" y="0" length="short" direction="pwr" rot="R90"/>
<pin name="PWR" x="1.27" y="0" length="short" direction="pwr" rot="R90"/>
<wire x1="0.254" y1="2.54" x2="2.286" y2="2.54" width="0.1524" layer="94"/>
<wire x1="2.286" y1="2.54" x2="2.286" y2="3.302" width="0.1524" layer="94"/>
<wire x1="2.286" y1="3.302" x2="2.032" y2="3.302" width="0.1524" layer="94"/>
<wire x1="2.032" y1="3.302" x2="0.508" y2="3.302" width="0.1524" layer="94"/>
<wire x1="0.508" y1="3.302" x2="0.254" y2="3.302" width="0.1524" layer="94"/>
<wire x1="0.254" y1="3.302" x2="0.254" y2="2.54" width="0.1524" layer="94"/>
<wire x1="0.508" y1="3.302" x2="0.508" y2="5.334" width="0.1524" layer="94"/>
<wire x1="2.032" y1="3.302" x2="2.032" y2="5.334" width="0.1524" layer="94"/>
<wire x1="0.508" y1="5.334" x2="2.032" y2="5.334" width="0.1524" layer="94" curve="-180"/>
<wire x1="-1.27" y1="2.54" x2="-1.27" y2="4.826" width="0.1524" layer="94"/>
<wire x1="-1.27" y1="4.826" x2="-0.508" y2="5.588" width="0.1524" layer="94"/>
<wire x1="-0.508" y1="5.588" x2="-1.27" y2="6.096" width="0.1524" layer="94"/>
<text x="4.318" y="2.794" size="1.778" layer="95" rot="R90">&gt;NAME</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="DCJACK">
<gates>
<gate name="J$1" symbol="DCJACK" x="0" y="-2.54"/>
</gates>
<devices>
<device name="" package="DCJACK">
<connects>
<connect gate="J$1" pin="GND" pad="2 3"/>
<connect gate="J$1" pin="PWR" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0.4572" drill="0.254">
<clearance class="0" value="0.205"/>
</class>
<class number="1" name="+18V" width="0.762" drill="0">
</class>
<class number="2" name="+5V" width="1.27" drill="0">
</class>
<class number="3" name="+3V3" width="1.27" drill="0">
</class>
<class number="4" name="DRV" width="0.762" drill="0">
</class>
<class number="5" name="GND" width="0.762" drill="0">
</class>
<class number="6" name="+12V" width="1.27" drill="0">
</class>
</classes>
<parts>
<part name="D1" library="diode" library_urn="urn:adsk.eagle:library:210" deviceset="1N4004" device="" package3d_urn="urn:adsk.eagle:package:43336/1" value="1N4001"/>
<part name="IC1" library="linear" library_urn="urn:adsk.eagle:library:262" deviceset="78*" device="L" package3d_urn="urn:adsk.eagle:package:16416/2" technology="12"/>
<part name="SVI1" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVI3" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA10-1" device="" package3d_urn="urn:adsk.eagle:package:8346/1"/>
<part name="SVI7" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA18-2" device="" package3d_urn="urn:adsk.eagle:package:8360/1"/>
<part name="VIN_ENABLE" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA03-1" device="" package3d_urn="urn:adsk.eagle:package:8339/1"/>
<part name="SVI2" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVO4" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVO5" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVO6" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVI6" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVI5" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVI4" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVO1" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVO2" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA08-1" device="" package3d_urn="urn:adsk.eagle:package:8343/1"/>
<part name="SVO3" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA10-1" device="" package3d_urn="urn:adsk.eagle:package:8346/1"/>
<part name="GND1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="3V3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device="" value="3V3"/>
<part name="+5V" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device="" value="V5"/>
<part name="GND3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND4" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device="" value="V5"/>
<part name="GND5" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND6" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND7" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="U1" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V4" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND9" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$3" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$4" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U2" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V6" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND10" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$5" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$6" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U3" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V7" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND11" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$7" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$8" library="ultrasonic" deviceset="TCT40-16" device=""/>
<part name="U4" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V8" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND12" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$9" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$10" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C1" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C2" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C3" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C4" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U5" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND8" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$1" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$2" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U6" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V9" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND13" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$11" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$12" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U7" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V10" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND14" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$13" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$14" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U8" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V11" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND15" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$15" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$16" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C5" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C6" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C7" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C8" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U9" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V12" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND16" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$17" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$18" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U10" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V13" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND17" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$19" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$20" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U11" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V14" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND18" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$21" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$22" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U12" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V15" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND19" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$23" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$24" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C9" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C10" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C11" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C12" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U13" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V16" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND20" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$25" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$26" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U14" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V17" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND21" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$27" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$28" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U15" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V18" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND22" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$29" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$30" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U16" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V19" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND23" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$31" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$32" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C13" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C14" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C15" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C16" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U17" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V20" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND24" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$33" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$34" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U18" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V21" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND25" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$35" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$36" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U19" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V22" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND26" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$37" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$38" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U20" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V23" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND27" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$39" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$40" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C17" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C18" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C19" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C20" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U21" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V24" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND28" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$41" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$42" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U22" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V25" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND29" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$43" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$44" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U23" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V26" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND30" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$45" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$46" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U24" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V27" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND31" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$47" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$48" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C21" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C22" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C23" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C24" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U25" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V28" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND32" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$49" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$50" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U26" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V29" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND33" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$51" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$52" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U27" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V30" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND34" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$53" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$54" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U28" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V31" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND35" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$55" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$56" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C25" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C26" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C27" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C28" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="U29" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V32" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND36" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$57" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$58" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U30" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V33" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND37" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$59" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$60" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U31" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V34" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND38" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$61" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$62" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U32" library="TC4427EOA" deviceset="TC4427EOA" device=""/>
<part name="V35" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="GND39" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="U$63" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="U$64" library="TCT40" deviceset="TCT40-16" device=""/>
<part name="C29" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C30" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C31" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C32" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="C33" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND40" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V36" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C34" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND41" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V37" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C35" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND42" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V38" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C36" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND43" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V39" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C37" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND44" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V40" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C38" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND45" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V41" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C39" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND46" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V42" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C40" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND47" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V43" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C41" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND48" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V44" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C42" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND49" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V45" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C43" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND50" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V46" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C44" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND51" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V47" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C45" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND52" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V48" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C46" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND53" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V49" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C47" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND54" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V50" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C48" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND55" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V51" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C49" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND56" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V52" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C50" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND57" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V53" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C51" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND58" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V54" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C52" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND59" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V55" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C53" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND60" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V56" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C54" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND61" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V57" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C55" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND62" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V58" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C56" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND63" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V59" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C57" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND64" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V60" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C58" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND65" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V61" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C59" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND66" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V62" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C60" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND67" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V63" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C61" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND68" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V64" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C62" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND69" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V65" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C63" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND70" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V66" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C64" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="C1206" package3d_urn="urn:adsk.eagle:package:23618/2" value="2.2uF"/>
<part name="GND71" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="V67" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+18V" device="" value="V20"/>
<part name="C65" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="050-025X075" package3d_urn="urn:adsk.eagle:package:23637/1" value="100pF"/>
<part name="C66" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-EU" device="050-025X075" package3d_urn="urn:adsk.eagle:package:23637/1" value="100pF"/>
<part name="RESET" library="switch-omron" library_urn="urn:adsk.eagle:library:377" deviceset="10-XX" device="" package3d_urn="urn:adsk.eagle:package:27496/1"/>
<part name="GND72" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="PWR_IN" library="DCJack" deviceset="DCJACK" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="D1" gate="1" x="27.94" y="119.38" smashed="yes">
<attribute name="NAME" x="30.48" y="119.8626" size="1.778" layer="95"/>
<attribute name="VALUE" x="30.48" y="117.0686" size="1.778" layer="96"/>
</instance>
<instance part="IC1" gate="A1" x="55.88" y="119.38" smashed="yes">
<attribute name="NAME" x="48.26" y="125.095" size="1.778" layer="95"/>
<attribute name="VALUE" x="48.26" y="122.555" size="1.778" layer="96"/>
</instance>
<instance part="SVI1" gate="1" x="88.9" y="-7.62" smashed="yes">
<attribute name="VALUE" x="87.63" y="-20.32" size="1.778" layer="96"/>
<attribute name="NAME" x="87.63" y="5.842" size="1.778" layer="95"/>
</instance>
<instance part="SVI3" gate="1" x="88.9" y="50.8" smashed="yes">
<attribute name="VALUE" x="87.63" y="35.56" size="1.778" layer="96"/>
<attribute name="NAME" x="87.63" y="66.802" size="1.778" layer="95"/>
</instance>
<instance part="SVI7" gate="G$1" x="63.5" y="-27.94" smashed="yes" rot="R90">
<attribute name="VALUE" x="88.9" y="-31.75" size="1.778" layer="96" rot="R90"/>
<attribute name="NAME" x="37.338" y="-31.75" size="1.778" layer="95" rot="R90"/>
</instance>
<instance part="VIN_ENABLE" gate="G$1" x="88.9" y="129.54" smashed="yes" rot="R270">
<attribute name="VALUE" x="81.28" y="130.81" size="1.778" layer="96" rot="R270"/>
<attribute name="NAME" x="94.742" y="130.81" size="1.778" layer="95" rot="R270"/>
</instance>
<instance part="SVI2" gate="1" x="88.9" y="20.32" smashed="yes">
<attribute name="VALUE" x="87.63" y="7.62" size="1.778" layer="96"/>
<attribute name="NAME" x="87.63" y="33.782" size="1.778" layer="95"/>
</instance>
<instance part="SVO4" gate="1" x="35.56" y="48.26" smashed="yes" rot="MR0">
<attribute name="VALUE" x="36.83" y="35.56" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="36.83" y="61.722" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="SVO5" gate="1" x="35.56" y="20.32" smashed="yes" rot="MR0">
<attribute name="VALUE" x="36.83" y="7.62" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="36.83" y="33.782" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="SVO6" gate="1" x="35.56" y="-7.62" smashed="yes" rot="MR0">
<attribute name="VALUE" x="36.83" y="-20.32" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="36.83" y="5.842" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="SVI6" gate="1" x="7.62" y="-7.62" smashed="yes">
<attribute name="VALUE" x="6.35" y="-20.32" size="1.778" layer="96"/>
<attribute name="NAME" x="6.35" y="5.842" size="1.778" layer="95"/>
</instance>
<instance part="SVI5" gate="1" x="7.62" y="20.32" smashed="yes">
<attribute name="VALUE" x="6.35" y="7.62" size="1.778" layer="96"/>
<attribute name="NAME" x="6.35" y="33.782" size="1.778" layer="95"/>
</instance>
<instance part="SVI4" gate="1" x="7.62" y="48.26" smashed="yes">
<attribute name="VALUE" x="6.35" y="35.56" size="1.778" layer="96"/>
<attribute name="NAME" x="6.35" y="61.722" size="1.778" layer="95"/>
</instance>
<instance part="SVO1" gate="1" x="116.84" y="-7.62" smashed="yes" rot="MR0">
<attribute name="VALUE" x="118.11" y="-20.32" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="118.11" y="5.842" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="SVO2" gate="1" x="116.84" y="20.32" smashed="yes" rot="MR0">
<attribute name="VALUE" x="118.11" y="7.62" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="118.11" y="33.782" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="SVO3" gate="1" x="116.84" y="50.8" smashed="yes" rot="MR0">
<attribute name="VALUE" x="118.11" y="35.56" size="1.778" layer="96" rot="MR0"/>
<attribute name="NAME" x="118.11" y="66.802" size="1.778" layer="95" rot="MR0"/>
</instance>
<instance part="GND1" gate="1" x="55.88" y="104.14" smashed="yes">
<attribute name="VALUE" x="53.34" y="101.6" size="1.778" layer="96"/>
</instance>
<instance part="GND2" gate="1" x="-7.62" y="35.56" smashed="yes">
<attribute name="VALUE" x="-10.16" y="33.02" size="1.778" layer="96"/>
</instance>
<instance part="3V3" gate="G$1" x="-2.54" y="60.96" smashed="yes">
<attribute name="VALUE" x="-2.54" y="60.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="+5V" gate="1" x="-7.62" y="60.96" smashed="yes">
<attribute name="VALUE" x="-7.62" y="60.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND3" gate="1" x="121.92" y="48.26" smashed="yes">
<attribute name="VALUE" x="119.38" y="45.72" size="1.778" layer="96"/>
</instance>
<instance part="GND4" gate="1" x="33.02" y="-53.34" smashed="yes">
<attribute name="VALUE" x="30.48" y="-55.88" size="1.778" layer="96"/>
</instance>
<instance part="V1" gate="1" x="96.52" y="-27.94" smashed="yes">
<attribute name="VALUE" x="96.52" y="-27.94" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND5" gate="1" x="10.16" y="119.38" smashed="yes">
<attribute name="VALUE" x="7.62" y="116.84" size="1.778" layer="96"/>
</instance>
<instance part="GND6" gate="1" x="40.64" y="104.14" smashed="yes">
<attribute name="VALUE" x="38.1" y="101.6" size="1.778" layer="96"/>
</instance>
<instance part="GND7" gate="1" x="73.66" y="104.14" smashed="yes">
<attribute name="VALUE" x="71.12" y="101.6" size="1.778" layer="96"/>
</instance>
<instance part="V2" gate="1" x="40.64" y="129.54" smashed="yes">
<attribute name="VALUE" x="40.64" y="129.54" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="U1" gate="A" x="177.8" y="129.54" smashed="yes">
<attribute name="NAME" x="165.1" y="143.24" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="112.84" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V4" gate="1" x="200.66" y="147.32" smashed="yes">
<attribute name="VALUE" x="200.66" y="147.32" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND9" gate="1" x="200.66" y="114.3" smashed="yes">
<attribute name="VALUE" x="198.12" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="U$3" gate="G$1" x="220.98" y="121.92" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="119.38" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="116.84" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$4" gate="G$1" x="220.98" y="137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="134.62" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="132.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U2" gate="A" x="177.8" y="83.82" smashed="yes">
<attribute name="NAME" x="165.1" y="97.52" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="67.12" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V6" gate="1" x="200.66" y="101.6" smashed="yes">
<attribute name="VALUE" x="200.66" y="101.6" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND10" gate="1" x="200.66" y="68.58" smashed="yes">
<attribute name="VALUE" x="198.12" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="U$5" gate="G$1" x="220.98" y="76.2" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="73.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="71.12" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$6" gate="G$1" x="220.98" y="91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="88.9" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="86.36" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U3" gate="A" x="177.8" y="38.1" smashed="yes">
<attribute name="NAME" x="165.1" y="51.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="21.4" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V7" gate="1" x="200.66" y="55.88" smashed="yes">
<attribute name="VALUE" x="200.66" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND11" gate="1" x="200.66" y="22.86" smashed="yes">
<attribute name="VALUE" x="198.12" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="U$7" gate="G$1" x="220.98" y="30.48" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="27.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="25.4" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$8" gate="G$1" x="220.98" y="45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="43.18" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="40.64" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U4" gate="A" x="177.8" y="-7.62" smashed="yes">
<attribute name="NAME" x="165.1" y="6.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="-24.32" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V8" gate="1" x="200.66" y="10.16" smashed="yes">
<attribute name="VALUE" x="200.66" y="10.16" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND12" gate="1" x="200.66" y="-22.86" smashed="yes">
<attribute name="VALUE" x="198.12" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="U$9" gate="G$1" x="220.98" y="-15.24" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-17.78" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-20.32" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$10" gate="G$1" x="220.98" y="0" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-2.54" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-5.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C1" gate="G$1" x="198.12" y="127" smashed="yes">
<attribute name="NAME" x="199.644" y="127.381" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="122.301" size="1.778" layer="96"/>
</instance>
<instance part="C2" gate="G$1" x="198.12" y="81.28" smashed="yes">
<attribute name="NAME" x="199.644" y="81.661" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="76.581" size="1.778" layer="96"/>
</instance>
<instance part="C3" gate="G$1" x="198.12" y="35.56" smashed="yes">
<attribute name="NAME" x="199.644" y="35.941" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="30.861" size="1.778" layer="96"/>
</instance>
<instance part="C4" gate="G$1" x="198.12" y="-10.16" smashed="yes">
<attribute name="NAME" x="199.644" y="-9.779" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="-14.859" size="1.778" layer="96"/>
</instance>
<instance part="U5" gate="A" x="177.8" y="-53.34" smashed="yes">
<attribute name="NAME" x="165.1" y="-39.64" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="-70.04" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V3" gate="1" x="200.66" y="-35.56" smashed="yes">
<attribute name="VALUE" x="200.66" y="-35.56" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND8" gate="1" x="200.66" y="-68.58" smashed="yes">
<attribute name="VALUE" x="198.12" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="U$1" gate="G$1" x="220.98" y="-60.96" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-63.5" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-66.04" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$2" gate="G$1" x="220.98" y="-45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-48.26" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-50.8" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U6" gate="A" x="177.8" y="-99.06" smashed="yes">
<attribute name="NAME" x="165.1" y="-85.36" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="-115.76" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V9" gate="1" x="200.66" y="-81.28" smashed="yes">
<attribute name="VALUE" x="200.66" y="-81.28" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND13" gate="1" x="200.66" y="-114.3" smashed="yes">
<attribute name="VALUE" x="198.12" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="U$11" gate="G$1" x="220.98" y="-106.68" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-109.22" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-111.76" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$12" gate="G$1" x="220.98" y="-91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-93.98" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-96.52" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U7" gate="A" x="177.8" y="-144.78" smashed="yes">
<attribute name="NAME" x="165.1" y="-131.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="-161.48" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V10" gate="1" x="200.66" y="-127" smashed="yes">
<attribute name="VALUE" x="200.66" y="-127" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND14" gate="1" x="200.66" y="-160.02" smashed="yes">
<attribute name="VALUE" x="198.12" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="U$13" gate="G$1" x="220.98" y="-152.4" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-154.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-157.48" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$14" gate="G$1" x="220.98" y="-137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-139.7" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-142.24" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U8" gate="A" x="177.8" y="-190.5" smashed="yes">
<attribute name="NAME" x="165.1" y="-176.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="165.1" y="-207.2" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V11" gate="1" x="200.66" y="-172.72" smashed="yes">
<attribute name="VALUE" x="200.66" y="-172.72" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND15" gate="1" x="200.66" y="-205.74" smashed="yes">
<attribute name="VALUE" x="198.12" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="U$15" gate="G$1" x="220.98" y="-198.12" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-200.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-203.2" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$16" gate="G$1" x="220.98" y="-182.88" smashed="yes" rot="MR0">
<attribute name="NAME" x="219.71" y="-185.42" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="219.71" y="-187.96" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C5" gate="G$1" x="198.12" y="-55.88" smashed="yes">
<attribute name="NAME" x="199.644" y="-55.499" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="-60.579" size="1.778" layer="96"/>
</instance>
<instance part="C6" gate="G$1" x="198.12" y="-101.6" smashed="yes">
<attribute name="NAME" x="199.644" y="-101.219" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="-106.299" size="1.778" layer="96"/>
</instance>
<instance part="C7" gate="G$1" x="198.12" y="-147.32" smashed="yes">
<attribute name="NAME" x="199.644" y="-146.939" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="-152.019" size="1.778" layer="96"/>
</instance>
<instance part="C8" gate="G$1" x="198.12" y="-193.04" smashed="yes">
<attribute name="NAME" x="199.644" y="-192.659" size="1.778" layer="95"/>
<attribute name="VALUE" x="199.644" y="-197.739" size="1.778" layer="96"/>
</instance>
<instance part="U9" gate="A" x="269.24" y="129.54" smashed="yes">
<attribute name="NAME" x="256.54" y="143.24" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="112.84" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V12" gate="1" x="292.1" y="147.32" smashed="yes">
<attribute name="VALUE" x="292.1" y="147.32" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND16" gate="1" x="292.1" y="114.3" smashed="yes">
<attribute name="VALUE" x="289.56" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="U$17" gate="G$1" x="312.42" y="121.92" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="119.38" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="116.84" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$18" gate="G$1" x="312.42" y="137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="134.62" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="132.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U10" gate="A" x="269.24" y="83.82" smashed="yes">
<attribute name="NAME" x="256.54" y="97.52" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="67.12" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V13" gate="1" x="292.1" y="101.6" smashed="yes">
<attribute name="VALUE" x="292.1" y="101.6" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND17" gate="1" x="292.1" y="68.58" smashed="yes">
<attribute name="VALUE" x="289.56" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="U$19" gate="G$1" x="312.42" y="76.2" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="73.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="71.12" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$20" gate="G$1" x="312.42" y="91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="88.9" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="86.36" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U11" gate="A" x="269.24" y="38.1" smashed="yes">
<attribute name="NAME" x="256.54" y="51.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="21.4" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V14" gate="1" x="292.1" y="55.88" smashed="yes">
<attribute name="VALUE" x="292.1" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND18" gate="1" x="292.1" y="22.86" smashed="yes">
<attribute name="VALUE" x="289.56" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="U$21" gate="G$1" x="312.42" y="30.48" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="27.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="25.4" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$22" gate="G$1" x="312.42" y="45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="43.18" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="40.64" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U12" gate="A" x="269.24" y="-7.62" smashed="yes">
<attribute name="NAME" x="256.54" y="6.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="-24.32" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V15" gate="1" x="292.1" y="10.16" smashed="yes">
<attribute name="VALUE" x="292.1" y="10.16" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND19" gate="1" x="292.1" y="-22.86" smashed="yes">
<attribute name="VALUE" x="289.56" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="U$23" gate="G$1" x="312.42" y="-15.24" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-17.78" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-20.32" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$24" gate="G$1" x="312.42" y="0" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-2.54" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-5.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C9" gate="G$1" x="289.56" y="127" smashed="yes">
<attribute name="NAME" x="291.084" y="127.381" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="122.301" size="1.778" layer="96"/>
</instance>
<instance part="C10" gate="G$1" x="289.56" y="81.28" smashed="yes">
<attribute name="NAME" x="291.084" y="81.661" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="76.581" size="1.778" layer="96"/>
</instance>
<instance part="C11" gate="G$1" x="289.56" y="35.56" smashed="yes">
<attribute name="NAME" x="291.084" y="35.941" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="30.861" size="1.778" layer="96"/>
</instance>
<instance part="C12" gate="G$1" x="289.56" y="-10.16" smashed="yes">
<attribute name="NAME" x="291.084" y="-9.779" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="-14.859" size="1.778" layer="96"/>
</instance>
<instance part="U13" gate="A" x="269.24" y="-53.34" smashed="yes">
<attribute name="NAME" x="256.54" y="-39.64" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="-70.04" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V16" gate="1" x="292.1" y="-35.56" smashed="yes">
<attribute name="VALUE" x="292.1" y="-35.56" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND20" gate="1" x="292.1" y="-68.58" smashed="yes">
<attribute name="VALUE" x="289.56" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="U$25" gate="G$1" x="312.42" y="-60.96" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-63.5" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-66.04" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$26" gate="G$1" x="312.42" y="-45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-48.26" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-50.8" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U14" gate="A" x="269.24" y="-99.06" smashed="yes">
<attribute name="NAME" x="256.54" y="-85.36" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="-115.76" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V17" gate="1" x="292.1" y="-81.28" smashed="yes">
<attribute name="VALUE" x="292.1" y="-81.28" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND21" gate="1" x="292.1" y="-114.3" smashed="yes">
<attribute name="VALUE" x="289.56" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="U$27" gate="G$1" x="312.42" y="-106.68" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-109.22" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-111.76" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$28" gate="G$1" x="312.42" y="-91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-93.98" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-96.52" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U15" gate="A" x="269.24" y="-144.78" smashed="yes">
<attribute name="NAME" x="256.54" y="-131.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="-161.48" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V18" gate="1" x="292.1" y="-127" smashed="yes">
<attribute name="VALUE" x="292.1" y="-127" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND22" gate="1" x="292.1" y="-160.02" smashed="yes">
<attribute name="VALUE" x="289.56" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="U$29" gate="G$1" x="312.42" y="-152.4" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-154.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-157.48" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$30" gate="G$1" x="312.42" y="-137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-139.7" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-142.24" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U16" gate="A" x="269.24" y="-190.5" smashed="yes">
<attribute name="NAME" x="256.54" y="-176.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="256.54" y="-207.2" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V19" gate="1" x="292.1" y="-172.72" smashed="yes">
<attribute name="VALUE" x="292.1" y="-172.72" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND23" gate="1" x="292.1" y="-205.74" smashed="yes">
<attribute name="VALUE" x="289.56" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="U$31" gate="G$1" x="312.42" y="-198.12" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-200.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-203.2" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$32" gate="G$1" x="312.42" y="-182.88" smashed="yes" rot="MR0">
<attribute name="NAME" x="311.15" y="-185.42" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="311.15" y="-187.96" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C13" gate="G$1" x="289.56" y="-55.88" smashed="yes">
<attribute name="NAME" x="291.084" y="-55.499" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="-60.579" size="1.778" layer="96"/>
</instance>
<instance part="C14" gate="G$1" x="289.56" y="-101.6" smashed="yes">
<attribute name="NAME" x="291.084" y="-101.219" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="-106.299" size="1.778" layer="96"/>
</instance>
<instance part="C15" gate="G$1" x="289.56" y="-147.32" smashed="yes">
<attribute name="NAME" x="291.084" y="-146.939" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="-152.019" size="1.778" layer="96"/>
</instance>
<instance part="C16" gate="G$1" x="289.56" y="-193.04" smashed="yes">
<attribute name="NAME" x="291.084" y="-192.659" size="1.778" layer="95"/>
<attribute name="VALUE" x="291.084" y="-197.739" size="1.778" layer="96"/>
</instance>
<instance part="U17" gate="A" x="358.14" y="129.54" smashed="yes">
<attribute name="NAME" x="345.44" y="143.24" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="112.84" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V20" gate="1" x="381" y="147.32" smashed="yes">
<attribute name="VALUE" x="381" y="147.32" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND24" gate="1" x="381" y="114.3" smashed="yes">
<attribute name="VALUE" x="378.46" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="U$33" gate="G$1" x="401.32" y="121.92" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="119.38" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="116.84" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$34" gate="G$1" x="401.32" y="137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="134.62" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="132.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U18" gate="A" x="358.14" y="83.82" smashed="yes">
<attribute name="NAME" x="345.44" y="97.52" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="67.12" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V21" gate="1" x="381" y="101.6" smashed="yes">
<attribute name="VALUE" x="381" y="101.6" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND25" gate="1" x="381" y="68.58" smashed="yes">
<attribute name="VALUE" x="378.46" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="U$35" gate="G$1" x="401.32" y="76.2" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="73.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="71.12" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$36" gate="G$1" x="401.32" y="91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="88.9" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="86.36" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U19" gate="A" x="358.14" y="38.1" smashed="yes">
<attribute name="NAME" x="345.44" y="51.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="21.4" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V22" gate="1" x="381" y="55.88" smashed="yes">
<attribute name="VALUE" x="381" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND26" gate="1" x="381" y="22.86" smashed="yes">
<attribute name="VALUE" x="378.46" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="U$37" gate="G$1" x="401.32" y="30.48" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="27.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="25.4" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$38" gate="G$1" x="401.32" y="45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="43.18" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="40.64" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U20" gate="A" x="358.14" y="-7.62" smashed="yes">
<attribute name="NAME" x="345.44" y="6.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="-24.32" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V23" gate="1" x="381" y="10.16" smashed="yes">
<attribute name="VALUE" x="381" y="10.16" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND27" gate="1" x="381" y="-22.86" smashed="yes">
<attribute name="VALUE" x="378.46" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="U$39" gate="G$1" x="401.32" y="-15.24" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-17.78" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-20.32" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$40" gate="G$1" x="401.32" y="0" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-2.54" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-5.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C17" gate="G$1" x="378.46" y="127" smashed="yes">
<attribute name="NAME" x="379.984" y="127.381" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="122.301" size="1.778" layer="96"/>
</instance>
<instance part="C18" gate="G$1" x="378.46" y="81.28" smashed="yes">
<attribute name="NAME" x="379.984" y="81.661" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="76.581" size="1.778" layer="96"/>
</instance>
<instance part="C19" gate="G$1" x="378.46" y="35.56" smashed="yes">
<attribute name="NAME" x="379.984" y="35.941" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="30.861" size="1.778" layer="96"/>
</instance>
<instance part="C20" gate="G$1" x="378.46" y="-10.16" smashed="yes">
<attribute name="NAME" x="379.984" y="-9.779" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="-14.859" size="1.778" layer="96"/>
</instance>
<instance part="U21" gate="A" x="358.14" y="-53.34" smashed="yes">
<attribute name="NAME" x="345.44" y="-39.64" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="-70.04" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V24" gate="1" x="381" y="-35.56" smashed="yes">
<attribute name="VALUE" x="381" y="-35.56" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND28" gate="1" x="381" y="-68.58" smashed="yes">
<attribute name="VALUE" x="378.46" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="U$41" gate="G$1" x="401.32" y="-60.96" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-63.5" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-66.04" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$42" gate="G$1" x="401.32" y="-45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-48.26" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-50.8" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U22" gate="A" x="358.14" y="-99.06" smashed="yes">
<attribute name="NAME" x="345.44" y="-85.36" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="-115.76" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V25" gate="1" x="381" y="-81.28" smashed="yes">
<attribute name="VALUE" x="381" y="-81.28" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND29" gate="1" x="381" y="-114.3" smashed="yes">
<attribute name="VALUE" x="378.46" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="U$43" gate="G$1" x="401.32" y="-106.68" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-109.22" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-111.76" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$44" gate="G$1" x="401.32" y="-91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-93.98" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-96.52" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U23" gate="A" x="358.14" y="-144.78" smashed="yes">
<attribute name="NAME" x="345.44" y="-131.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="-161.48" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V26" gate="1" x="381" y="-127" smashed="yes">
<attribute name="VALUE" x="381" y="-127" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND30" gate="1" x="381" y="-160.02" smashed="yes">
<attribute name="VALUE" x="378.46" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="U$45" gate="G$1" x="401.32" y="-152.4" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-154.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-157.48" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$46" gate="G$1" x="401.32" y="-137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-139.7" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-142.24" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U24" gate="A" x="358.14" y="-190.5" smashed="yes">
<attribute name="NAME" x="345.44" y="-176.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="345.44" y="-207.2" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V27" gate="1" x="381" y="-172.72" smashed="yes">
<attribute name="VALUE" x="381" y="-172.72" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND31" gate="1" x="381" y="-205.74" smashed="yes">
<attribute name="VALUE" x="378.46" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="U$47" gate="G$1" x="401.32" y="-198.12" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-200.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-203.2" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$48" gate="G$1" x="401.32" y="-182.88" smashed="yes" rot="MR0">
<attribute name="NAME" x="400.05" y="-185.42" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="400.05" y="-187.96" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C21" gate="G$1" x="378.46" y="-55.88" smashed="yes">
<attribute name="NAME" x="379.984" y="-55.499" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="-60.579" size="1.778" layer="96"/>
</instance>
<instance part="C22" gate="G$1" x="378.46" y="-101.6" smashed="yes">
<attribute name="NAME" x="379.984" y="-101.219" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="-106.299" size="1.778" layer="96"/>
</instance>
<instance part="C23" gate="G$1" x="378.46" y="-147.32" smashed="yes">
<attribute name="NAME" x="379.984" y="-146.939" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="-152.019" size="1.778" layer="96"/>
</instance>
<instance part="C24" gate="G$1" x="378.46" y="-193.04" smashed="yes">
<attribute name="NAME" x="379.984" y="-192.659" size="1.778" layer="95"/>
<attribute name="VALUE" x="379.984" y="-197.739" size="1.778" layer="96"/>
</instance>
<instance part="U25" gate="A" x="447.04" y="129.54" smashed="yes">
<attribute name="NAME" x="434.34" y="143.24" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="112.84" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V28" gate="1" x="469.9" y="147.32" smashed="yes">
<attribute name="VALUE" x="469.9" y="147.32" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND32" gate="1" x="469.9" y="114.3" smashed="yes">
<attribute name="VALUE" x="467.36" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="U$49" gate="G$1" x="490.22" y="121.92" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="119.38" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="116.84" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$50" gate="G$1" x="490.22" y="137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="134.62" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="132.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U26" gate="A" x="447.04" y="83.82" smashed="yes">
<attribute name="NAME" x="434.34" y="97.52" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="67.12" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V29" gate="1" x="469.9" y="101.6" smashed="yes">
<attribute name="VALUE" x="469.9" y="101.6" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND33" gate="1" x="469.9" y="68.58" smashed="yes">
<attribute name="VALUE" x="467.36" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="U$51" gate="G$1" x="490.22" y="76.2" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="73.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="71.12" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$52" gate="G$1" x="490.22" y="91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="88.9" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="86.36" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U27" gate="A" x="447.04" y="38.1" smashed="yes">
<attribute name="NAME" x="434.34" y="51.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="21.4" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V30" gate="1" x="469.9" y="55.88" smashed="yes">
<attribute name="VALUE" x="469.9" y="55.88" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND34" gate="1" x="469.9" y="22.86" smashed="yes">
<attribute name="VALUE" x="467.36" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="U$53" gate="G$1" x="490.22" y="30.48" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="27.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="25.4" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$54" gate="G$1" x="490.22" y="45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="43.18" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="40.64" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U28" gate="A" x="447.04" y="-7.62" smashed="yes">
<attribute name="NAME" x="434.34" y="6.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="-24.32" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V31" gate="1" x="469.9" y="10.16" smashed="yes">
<attribute name="VALUE" x="469.9" y="10.16" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND35" gate="1" x="469.9" y="-22.86" smashed="yes">
<attribute name="VALUE" x="467.36" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="U$55" gate="G$1" x="490.22" y="-15.24" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-17.78" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-20.32" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$56" gate="G$1" x="490.22" y="0" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-2.54" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-5.08" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C25" gate="G$1" x="467.36" y="127" smashed="yes">
<attribute name="NAME" x="468.884" y="127.381" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="122.301" size="1.778" layer="96"/>
</instance>
<instance part="C26" gate="G$1" x="467.36" y="81.28" smashed="yes">
<attribute name="NAME" x="468.884" y="81.661" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="76.581" size="1.778" layer="96"/>
</instance>
<instance part="C27" gate="G$1" x="467.36" y="35.56" smashed="yes">
<attribute name="NAME" x="468.884" y="35.941" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="30.861" size="1.778" layer="96"/>
</instance>
<instance part="C28" gate="G$1" x="467.36" y="-10.16" smashed="yes">
<attribute name="NAME" x="468.884" y="-9.779" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="-14.859" size="1.778" layer="96"/>
</instance>
<instance part="U29" gate="A" x="447.04" y="-53.34" smashed="yes">
<attribute name="NAME" x="434.34" y="-39.64" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="-70.04" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V32" gate="1" x="469.9" y="-35.56" smashed="yes">
<attribute name="VALUE" x="469.9" y="-35.56" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND36" gate="1" x="469.9" y="-68.58" smashed="yes">
<attribute name="VALUE" x="467.36" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="U$57" gate="G$1" x="490.22" y="-60.96" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-63.5" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-66.04" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$58" gate="G$1" x="490.22" y="-45.72" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-48.26" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-50.8" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U30" gate="A" x="447.04" y="-99.06" smashed="yes">
<attribute name="NAME" x="434.34" y="-85.36" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="-115.76" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V33" gate="1" x="469.9" y="-81.28" smashed="yes">
<attribute name="VALUE" x="469.9" y="-81.28" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND37" gate="1" x="469.9" y="-114.3" smashed="yes">
<attribute name="VALUE" x="467.36" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="U$59" gate="G$1" x="490.22" y="-106.68" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-109.22" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-111.76" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$60" gate="G$1" x="490.22" y="-91.44" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-93.98" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-96.52" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U31" gate="A" x="447.04" y="-144.78" smashed="yes">
<attribute name="NAME" x="434.34" y="-131.08" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="-161.48" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V34" gate="1" x="469.9" y="-127" smashed="yes">
<attribute name="VALUE" x="469.9" y="-127" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND38" gate="1" x="469.9" y="-160.02" smashed="yes">
<attribute name="VALUE" x="467.36" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="U$61" gate="G$1" x="490.22" y="-152.4" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-154.94" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-157.48" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$62" gate="G$1" x="490.22" y="-137.16" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-139.7" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-142.24" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U32" gate="A" x="447.04" y="-190.5" smashed="yes">
<attribute name="NAME" x="434.34" y="-176.8" size="2.0828" layer="95" ratio="10" rot="SR0"/>
<attribute name="VALUE" x="434.34" y="-207.2" size="2.0828" layer="96" ratio="10" rot="SR0"/>
</instance>
<instance part="V35" gate="1" x="469.9" y="-172.72" smashed="yes">
<attribute name="VALUE" x="469.9" y="-172.72" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND39" gate="1" x="469.9" y="-205.74" smashed="yes">
<attribute name="VALUE" x="467.36" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="U$63" gate="G$1" x="490.22" y="-198.12" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-200.66" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-203.2" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="U$64" gate="G$1" x="490.22" y="-182.88" smashed="yes" rot="MR0">
<attribute name="NAME" x="488.95" y="-185.42" size="1.27" layer="95" rot="MR0"/>
<attribute name="VALUE" x="488.95" y="-187.96" size="1.27" layer="96" rot="MR0"/>
</instance>
<instance part="C29" gate="G$1" x="467.36" y="-55.88" smashed="yes">
<attribute name="NAME" x="468.884" y="-55.499" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="-60.579" size="1.778" layer="96"/>
</instance>
<instance part="C30" gate="G$1" x="467.36" y="-101.6" smashed="yes">
<attribute name="NAME" x="468.884" y="-101.219" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="-106.299" size="1.778" layer="96"/>
</instance>
<instance part="C31" gate="G$1" x="467.36" y="-147.32" smashed="yes">
<attribute name="NAME" x="468.884" y="-146.939" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="-152.019" size="1.778" layer="96"/>
</instance>
<instance part="C32" gate="G$1" x="467.36" y="-193.04" smashed="yes">
<attribute name="NAME" x="468.884" y="-192.659" size="1.778" layer="95"/>
<attribute name="VALUE" x="468.884" y="-197.739" size="1.778" layer="96"/>
</instance>
<instance part="C33" gate="G$1" x="228.6" y="124.46" smashed="yes">
<attribute name="NAME" x="230.124" y="124.841" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="119.761" size="1.778" layer="96"/>
</instance>
<instance part="GND40" gate="1" x="228.6" y="114.3" smashed="yes">
<attribute name="VALUE" x="226.06" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="V36" gate="1" x="228.6" y="132.08" smashed="yes">
<attribute name="VALUE" x="228.6" y="132.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C34" gate="G$1" x="228.6" y="78.74" smashed="yes">
<attribute name="NAME" x="230.124" y="79.121" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="74.041" size="1.778" layer="96"/>
</instance>
<instance part="GND41" gate="1" x="228.6" y="68.58" smashed="yes">
<attribute name="VALUE" x="226.06" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="V37" gate="1" x="228.6" y="86.36" smashed="yes">
<attribute name="VALUE" x="228.6" y="86.36" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C35" gate="G$1" x="228.6" y="33.02" smashed="yes">
<attribute name="NAME" x="230.124" y="33.401" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="28.321" size="1.778" layer="96"/>
</instance>
<instance part="GND42" gate="1" x="228.6" y="22.86" smashed="yes">
<attribute name="VALUE" x="226.06" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="V38" gate="1" x="228.6" y="40.64" smashed="yes">
<attribute name="VALUE" x="228.6" y="40.64" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C36" gate="G$1" x="228.6" y="-12.7" smashed="yes">
<attribute name="NAME" x="230.124" y="-12.319" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="-17.399" size="1.778" layer="96"/>
</instance>
<instance part="GND43" gate="1" x="228.6" y="-22.86" smashed="yes">
<attribute name="VALUE" x="226.06" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="V39" gate="1" x="228.6" y="-5.08" smashed="yes">
<attribute name="VALUE" x="228.6" y="-5.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C37" gate="G$1" x="228.6" y="-58.42" smashed="yes">
<attribute name="NAME" x="230.124" y="-58.039" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="-63.119" size="1.778" layer="96"/>
</instance>
<instance part="GND44" gate="1" x="228.6" y="-68.58" smashed="yes">
<attribute name="VALUE" x="226.06" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="V40" gate="1" x="228.6" y="-50.8" smashed="yes">
<attribute name="VALUE" x="228.6" y="-50.8" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C38" gate="G$1" x="228.6" y="-104.14" smashed="yes">
<attribute name="NAME" x="230.124" y="-103.759" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="-108.839" size="1.778" layer="96"/>
</instance>
<instance part="GND45" gate="1" x="228.6" y="-114.3" smashed="yes">
<attribute name="VALUE" x="226.06" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="V41" gate="1" x="228.6" y="-96.52" smashed="yes">
<attribute name="VALUE" x="228.6" y="-96.52" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C39" gate="G$1" x="228.6" y="-149.86" smashed="yes">
<attribute name="NAME" x="230.124" y="-149.479" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="-154.559" size="1.778" layer="96"/>
</instance>
<instance part="GND46" gate="1" x="228.6" y="-160.02" smashed="yes">
<attribute name="VALUE" x="226.06" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="V42" gate="1" x="228.6" y="-142.24" smashed="yes">
<attribute name="VALUE" x="228.6" y="-142.24" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C40" gate="G$1" x="228.6" y="-195.58" smashed="yes">
<attribute name="NAME" x="230.124" y="-195.199" size="1.778" layer="95"/>
<attribute name="VALUE" x="230.124" y="-200.279" size="1.778" layer="96"/>
</instance>
<instance part="GND47" gate="1" x="228.6" y="-205.74" smashed="yes">
<attribute name="VALUE" x="226.06" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="V43" gate="1" x="228.6" y="-187.96" smashed="yes">
<attribute name="VALUE" x="228.6" y="-187.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C41" gate="G$1" x="320.04" y="124.46" smashed="yes">
<attribute name="NAME" x="321.564" y="124.841" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="119.761" size="1.778" layer="96"/>
</instance>
<instance part="GND48" gate="1" x="320.04" y="114.3" smashed="yes">
<attribute name="VALUE" x="317.5" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="V44" gate="1" x="320.04" y="132.08" smashed="yes">
<attribute name="VALUE" x="320.04" y="132.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C42" gate="G$1" x="320.04" y="78.74" smashed="yes">
<attribute name="NAME" x="321.564" y="79.121" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="74.041" size="1.778" layer="96"/>
</instance>
<instance part="GND49" gate="1" x="320.04" y="68.58" smashed="yes">
<attribute name="VALUE" x="317.5" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="V45" gate="1" x="320.04" y="86.36" smashed="yes">
<attribute name="VALUE" x="320.04" y="86.36" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C43" gate="G$1" x="320.04" y="33.02" smashed="yes">
<attribute name="NAME" x="321.564" y="33.401" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="28.321" size="1.778" layer="96"/>
</instance>
<instance part="GND50" gate="1" x="320.04" y="22.86" smashed="yes">
<attribute name="VALUE" x="317.5" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="V46" gate="1" x="320.04" y="40.64" smashed="yes">
<attribute name="VALUE" x="320.04" y="40.64" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C44" gate="G$1" x="320.04" y="-12.7" smashed="yes">
<attribute name="NAME" x="321.564" y="-12.319" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="-17.399" size="1.778" layer="96"/>
</instance>
<instance part="GND51" gate="1" x="320.04" y="-22.86" smashed="yes">
<attribute name="VALUE" x="317.5" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="V47" gate="1" x="320.04" y="-5.08" smashed="yes">
<attribute name="VALUE" x="320.04" y="-5.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C45" gate="G$1" x="320.04" y="-58.42" smashed="yes">
<attribute name="NAME" x="321.564" y="-58.039" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="-63.119" size="1.778" layer="96"/>
</instance>
<instance part="GND52" gate="1" x="320.04" y="-68.58" smashed="yes">
<attribute name="VALUE" x="317.5" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="V48" gate="1" x="320.04" y="-50.8" smashed="yes">
<attribute name="VALUE" x="320.04" y="-50.8" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C46" gate="G$1" x="320.04" y="-104.14" smashed="yes">
<attribute name="NAME" x="321.564" y="-103.759" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="-108.839" size="1.778" layer="96"/>
</instance>
<instance part="GND53" gate="1" x="320.04" y="-114.3" smashed="yes">
<attribute name="VALUE" x="317.5" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="V49" gate="1" x="320.04" y="-96.52" smashed="yes">
<attribute name="VALUE" x="320.04" y="-96.52" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C47" gate="G$1" x="320.04" y="-149.86" smashed="yes">
<attribute name="NAME" x="321.564" y="-149.479" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="-154.559" size="1.778" layer="96"/>
</instance>
<instance part="GND54" gate="1" x="320.04" y="-160.02" smashed="yes">
<attribute name="VALUE" x="317.5" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="V50" gate="1" x="320.04" y="-142.24" smashed="yes">
<attribute name="VALUE" x="320.04" y="-142.24" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C48" gate="G$1" x="320.04" y="-195.58" smashed="yes">
<attribute name="NAME" x="321.564" y="-195.199" size="1.778" layer="95"/>
<attribute name="VALUE" x="321.564" y="-200.279" size="1.778" layer="96"/>
</instance>
<instance part="GND55" gate="1" x="320.04" y="-205.74" smashed="yes">
<attribute name="VALUE" x="317.5" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="V51" gate="1" x="320.04" y="-187.96" smashed="yes">
<attribute name="VALUE" x="320.04" y="-187.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C49" gate="G$1" x="408.94" y="124.46" smashed="yes">
<attribute name="NAME" x="410.464" y="124.841" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="119.761" size="1.778" layer="96"/>
</instance>
<instance part="GND56" gate="1" x="408.94" y="114.3" smashed="yes">
<attribute name="VALUE" x="406.4" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="V52" gate="1" x="408.94" y="132.08" smashed="yes">
<attribute name="VALUE" x="408.94" y="132.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C50" gate="G$1" x="408.94" y="78.74" smashed="yes">
<attribute name="NAME" x="410.464" y="79.121" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="74.041" size="1.778" layer="96"/>
</instance>
<instance part="GND57" gate="1" x="408.94" y="68.58" smashed="yes">
<attribute name="VALUE" x="406.4" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="V53" gate="1" x="408.94" y="86.36" smashed="yes">
<attribute name="VALUE" x="408.94" y="86.36" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C51" gate="G$1" x="408.94" y="33.02" smashed="yes">
<attribute name="NAME" x="410.464" y="33.401" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="28.321" size="1.778" layer="96"/>
</instance>
<instance part="GND58" gate="1" x="408.94" y="22.86" smashed="yes">
<attribute name="VALUE" x="406.4" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="V54" gate="1" x="408.94" y="40.64" smashed="yes">
<attribute name="VALUE" x="408.94" y="40.64" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C52" gate="G$1" x="408.94" y="-12.7" smashed="yes">
<attribute name="NAME" x="410.464" y="-12.319" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="-17.399" size="1.778" layer="96"/>
</instance>
<instance part="GND59" gate="1" x="408.94" y="-22.86" smashed="yes">
<attribute name="VALUE" x="406.4" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="V55" gate="1" x="408.94" y="-5.08" smashed="yes">
<attribute name="VALUE" x="408.94" y="-5.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C53" gate="G$1" x="408.94" y="-58.42" smashed="yes">
<attribute name="NAME" x="410.464" y="-58.039" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="-63.119" size="1.778" layer="96"/>
</instance>
<instance part="GND60" gate="1" x="408.94" y="-68.58" smashed="yes">
<attribute name="VALUE" x="406.4" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="V56" gate="1" x="408.94" y="-50.8" smashed="yes">
<attribute name="VALUE" x="408.94" y="-50.8" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C54" gate="G$1" x="408.94" y="-104.14" smashed="yes">
<attribute name="NAME" x="410.464" y="-103.759" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="-108.839" size="1.778" layer="96"/>
</instance>
<instance part="GND61" gate="1" x="408.94" y="-114.3" smashed="yes">
<attribute name="VALUE" x="406.4" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="V57" gate="1" x="408.94" y="-96.52" smashed="yes">
<attribute name="VALUE" x="408.94" y="-96.52" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C55" gate="G$1" x="408.94" y="-149.86" smashed="yes">
<attribute name="NAME" x="410.464" y="-149.479" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="-154.559" size="1.778" layer="96"/>
</instance>
<instance part="GND62" gate="1" x="408.94" y="-160.02" smashed="yes">
<attribute name="VALUE" x="406.4" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="V58" gate="1" x="408.94" y="-142.24" smashed="yes">
<attribute name="VALUE" x="408.94" y="-142.24" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C56" gate="G$1" x="408.94" y="-195.58" smashed="yes">
<attribute name="NAME" x="410.464" y="-195.199" size="1.778" layer="95"/>
<attribute name="VALUE" x="410.464" y="-200.279" size="1.778" layer="96"/>
</instance>
<instance part="GND63" gate="1" x="408.94" y="-205.74" smashed="yes">
<attribute name="VALUE" x="406.4" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="V59" gate="1" x="408.94" y="-187.96" smashed="yes">
<attribute name="VALUE" x="408.94" y="-187.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C57" gate="G$1" x="497.84" y="124.46" smashed="yes">
<attribute name="NAME" x="499.364" y="124.841" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="119.761" size="1.778" layer="96"/>
</instance>
<instance part="GND64" gate="1" x="497.84" y="114.3" smashed="yes">
<attribute name="VALUE" x="495.3" y="111.76" size="1.778" layer="96"/>
</instance>
<instance part="V60" gate="1" x="497.84" y="132.08" smashed="yes">
<attribute name="VALUE" x="497.84" y="132.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C58" gate="G$1" x="497.84" y="78.74" smashed="yes">
<attribute name="NAME" x="499.364" y="79.121" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="74.041" size="1.778" layer="96"/>
</instance>
<instance part="GND65" gate="1" x="497.84" y="68.58" smashed="yes">
<attribute name="VALUE" x="495.3" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="V61" gate="1" x="497.84" y="86.36" smashed="yes">
<attribute name="VALUE" x="497.84" y="86.36" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C59" gate="G$1" x="497.84" y="33.02" smashed="yes">
<attribute name="NAME" x="499.364" y="33.401" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="28.321" size="1.778" layer="96"/>
</instance>
<instance part="GND66" gate="1" x="497.84" y="22.86" smashed="yes">
<attribute name="VALUE" x="495.3" y="20.32" size="1.778" layer="96"/>
</instance>
<instance part="V62" gate="1" x="497.84" y="40.64" smashed="yes">
<attribute name="VALUE" x="497.84" y="40.64" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C60" gate="G$1" x="497.84" y="-12.7" smashed="yes">
<attribute name="NAME" x="499.364" y="-12.319" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="-17.399" size="1.778" layer="96"/>
</instance>
<instance part="GND67" gate="1" x="497.84" y="-22.86" smashed="yes">
<attribute name="VALUE" x="495.3" y="-25.4" size="1.778" layer="96"/>
</instance>
<instance part="V63" gate="1" x="497.84" y="-5.08" smashed="yes">
<attribute name="VALUE" x="497.84" y="-5.08" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C61" gate="G$1" x="497.84" y="-58.42" smashed="yes">
<attribute name="NAME" x="499.364" y="-58.039" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="-63.119" size="1.778" layer="96"/>
</instance>
<instance part="GND68" gate="1" x="497.84" y="-68.58" smashed="yes">
<attribute name="VALUE" x="495.3" y="-71.12" size="1.778" layer="96"/>
</instance>
<instance part="V64" gate="1" x="497.84" y="-50.8" smashed="yes">
<attribute name="VALUE" x="497.84" y="-50.8" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C62" gate="G$1" x="497.84" y="-104.14" smashed="yes">
<attribute name="NAME" x="499.364" y="-103.759" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="-108.839" size="1.778" layer="96"/>
</instance>
<instance part="GND69" gate="1" x="497.84" y="-114.3" smashed="yes">
<attribute name="VALUE" x="495.3" y="-116.84" size="1.778" layer="96"/>
</instance>
<instance part="V65" gate="1" x="497.84" y="-96.52" smashed="yes">
<attribute name="VALUE" x="497.84" y="-96.52" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C63" gate="G$1" x="497.84" y="-149.86" smashed="yes">
<attribute name="NAME" x="499.364" y="-149.479" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="-154.559" size="1.778" layer="96"/>
</instance>
<instance part="GND70" gate="1" x="497.84" y="-160.02" smashed="yes">
<attribute name="VALUE" x="495.3" y="-162.56" size="1.778" layer="96"/>
</instance>
<instance part="V66" gate="1" x="497.84" y="-142.24" smashed="yes">
<attribute name="VALUE" x="497.84" y="-142.24" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C64" gate="G$1" x="497.84" y="-195.58" smashed="yes">
<attribute name="NAME" x="499.364" y="-195.199" size="1.778" layer="95"/>
<attribute name="VALUE" x="499.364" y="-200.279" size="1.778" layer="96"/>
</instance>
<instance part="GND71" gate="1" x="497.84" y="-205.74" smashed="yes">
<attribute name="VALUE" x="495.3" y="-208.28" size="1.778" layer="96"/>
</instance>
<instance part="V67" gate="1" x="497.84" y="-187.96" smashed="yes">
<attribute name="VALUE" x="497.84" y="-187.96" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C65" gate="G$1" x="40.64" y="114.3" smashed="yes">
<attribute name="NAME" x="42.164" y="114.681" size="1.778" layer="95"/>
<attribute name="VALUE" x="42.164" y="109.601" size="1.778" layer="96"/>
</instance>
<instance part="C66" gate="G$1" x="73.66" y="114.3" smashed="yes">
<attribute name="NAME" x="75.184" y="114.681" size="1.778" layer="95"/>
<attribute name="VALUE" x="75.184" y="109.601" size="1.778" layer="96"/>
</instance>
<instance part="RESET" gate="1" x="-5.08" y="104.14" smashed="yes" rot="R270">
<attribute name="NAME" x="-7.62" y="110.49" size="1.778" layer="95"/>
<attribute name="VALUE" x="-1.905" y="107.95" size="1.778" layer="96"/>
</instance>
<instance part="GND72" gate="1" x="12.7" y="96.52" smashed="yes">
<attribute name="VALUE" x="10.16" y="93.98" size="1.778" layer="96"/>
</instance>
<instance part="PWR_IN" gate="J$1" x="15.24" y="127" smashed="yes">
<attribute name="NAME" x="19.558" y="129.794" size="1.778" layer="95" rot="R90"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="PA0" class="0">
<segment>
<wire x1="96.52" y1="12.7" x2="109.22" y2="12.7" width="0.1524" layer="91"/>
<label x="101.6" y="12.7" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="1"/>
<pinref part="SVO2" gate="1" pin="1"/>
</segment>
</net>
<net name="PA1" class="0">
<segment>
<wire x1="96.52" y1="15.24" x2="109.22" y2="15.24" width="0.1524" layer="91"/>
<label x="101.6" y="15.24" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="2"/>
<pinref part="SVO2" gate="1" pin="2"/>
</segment>
</net>
<net name="PA2" class="0">
<segment>
<wire x1="96.52" y1="17.78" x2="109.22" y2="17.78" width="0.1524" layer="91"/>
<label x="101.6" y="17.78" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="3"/>
<pinref part="SVO2" gate="1" pin="3"/>
</segment>
</net>
<net name="PA3" class="0">
<segment>
<wire x1="96.52" y1="20.32" x2="109.22" y2="20.32" width="0.1524" layer="91"/>
<label x="101.6" y="20.32" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="4"/>
<pinref part="SVO2" gate="1" pin="4"/>
</segment>
</net>
<net name="PA4" class="0">
<segment>
<wire x1="96.52" y1="22.86" x2="109.22" y2="22.86" width="0.1524" layer="91"/>
<label x="101.6" y="22.86" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="5"/>
<pinref part="SVO2" gate="1" pin="5"/>
</segment>
</net>
<net name="PA5" class="0">
<segment>
<wire x1="96.52" y1="25.4" x2="109.22" y2="25.4" width="0.1524" layer="91"/>
<label x="101.6" y="25.4" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="6"/>
<pinref part="SVO2" gate="1" pin="6"/>
</segment>
</net>
<net name="PA6" class="0">
<segment>
<wire x1="96.52" y1="27.94" x2="109.22" y2="27.94" width="0.1524" layer="91"/>
<label x="101.6" y="27.94" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="7"/>
<pinref part="SVO2" gate="1" pin="7"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="132.08" x2="144.78" y2="132.08" width="0.1524" layer="91"/>
<label x="147.32" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA7" class="0">
<segment>
<wire x1="96.52" y1="30.48" x2="109.22" y2="30.48" width="0.1524" layer="91"/>
<label x="101.6" y="30.48" size="1.778" layer="95"/>
<pinref part="SVI2" gate="1" pin="8"/>
<pinref part="SVO2" gate="1" pin="8"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="134.62" x2="144.78" y2="134.62" width="0.1524" layer="91"/>
<label x="147.32" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA8" class="0">
<segment>
<wire x1="96.52" y1="40.64" x2="109.22" y2="40.64" width="0.1524" layer="91"/>
<label x="101.6" y="40.64" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="1"/>
<pinref part="SVO3" gate="1" pin="1"/>
</segment>
<segment>
<pinref part="U2" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="86.36" x2="144.78" y2="86.36" width="0.1524" layer="91"/>
<label x="147.32" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA9" class="0">
<segment>
<wire x1="96.52" y1="43.18" x2="109.22" y2="43.18" width="0.1524" layer="91"/>
<label x="101.6" y="43.18" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="2"/>
<pinref part="SVO3" gate="1" pin="2"/>
</segment>
<segment>
<pinref part="U2" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="88.9" x2="144.78" y2="88.9" width="0.1524" layer="91"/>
<label x="147.32" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA10" class="0">
<segment>
<wire x1="96.52" y1="45.72" x2="109.22" y2="45.72" width="0.1524" layer="91"/>
<label x="101.6" y="45.72" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="3"/>
<pinref part="SVO3" gate="1" pin="3"/>
</segment>
<segment>
<pinref part="U3" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="43.18" x2="144.78" y2="43.18" width="0.1524" layer="91"/>
<label x="147.32" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA11" class="0">
<segment>
<wire x1="96.52" y1="48.26" x2="109.22" y2="48.26" width="0.1524" layer="91"/>
<label x="101.6" y="48.26" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="4"/>
<pinref part="SVO3" gate="1" pin="4"/>
</segment>
<segment>
<pinref part="U3" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="40.64" x2="144.78" y2="40.64" width="0.1524" layer="91"/>
<label x="147.32" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA12" class="0">
<segment>
<wire x1="96.52" y1="50.8" x2="109.22" y2="50.8" width="0.1524" layer="91"/>
<label x="101.6" y="50.8" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="5"/>
<pinref part="SVO3" gate="1" pin="5"/>
</segment>
<segment>
<pinref part="U4" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="-2.54" x2="144.78" y2="-2.54" width="0.1524" layer="91"/>
<label x="147.32" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA13" class="0">
<segment>
<wire x1="96.52" y1="53.34" x2="109.22" y2="53.34" width="0.1524" layer="91"/>
<label x="101.6" y="53.34" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="6"/>
<pinref part="SVO3" gate="1" pin="6"/>
</segment>
<segment>
<pinref part="U4" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="-5.08" x2="144.78" y2="-5.08" width="0.1524" layer="91"/>
<label x="147.32" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="GND" class="5">
<segment>
<wire x1="96.52" y1="55.88" x2="109.22" y2="55.88" width="0.1524" layer="91"/>
<label x="101.6" y="55.88" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="7"/>
<pinref part="SVO3" gate="1" pin="7"/>
<wire x1="109.22" y1="55.88" x2="121.92" y2="55.88" width="0.1524" layer="91"/>
<junction x="109.22" y="55.88"/>
<wire x1="121.92" y1="55.88" x2="121.92" y2="50.8" width="0.1524" layer="91"/>
<pinref part="GND3" gate="1" pin="GND"/>
</segment>
<segment>
<wire x1="40.64" y1="-20.32" x2="40.64" y2="-2.54" width="0.1524" layer="91"/>
<label x="40.64" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="35"/>
<wire x1="40.64" y1="-20.32" x2="33.02" y2="-20.32" width="0.1524" layer="91"/>
<junction x="40.64" y="-20.32"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="33.02" y1="-20.32" x2="33.02" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="33.02" y1="-35.56" x2="33.02" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="40.64" y1="-48.26" x2="40.64" y2="-35.56" width="0.1524" layer="91"/>
<label x="40.64" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="36"/>
<wire x1="40.64" y1="-35.56" x2="33.02" y2="-35.56" width="0.1524" layer="91"/>
<junction x="40.64" y="-35.56"/>
<junction x="33.02" y="-35.56"/>
</segment>
<segment>
<wire x1="15.24" y1="45.72" x2="27.94" y2="45.72" width="0.1524" layer="91"/>
<label x="17.78" y="45.72" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="3"/>
<pinref part="SVI4" gate="1" pin="3"/>
<wire x1="15.24" y1="45.72" x2="-7.62" y2="45.72" width="0.1524" layer="91"/>
<junction x="15.24" y="45.72"/>
<wire x1="-7.62" y1="45.72" x2="-7.62" y2="43.18" width="0.1524" layer="91"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="-7.62" y1="43.18" x2="-7.62" y2="38.1" width="0.1524" layer="91"/>
<wire x1="15.24" y1="43.18" x2="27.94" y2="43.18" width="0.1524" layer="91"/>
<label x="17.78" y="43.18" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="2"/>
<pinref part="SVI4" gate="1" pin="2"/>
<wire x1="15.24" y1="43.18" x2="-7.62" y2="43.18" width="0.1524" layer="91"/>
<junction x="15.24" y="43.18"/>
<junction x="-7.62" y="43.18"/>
</segment>
<segment>
<pinref part="IC1" gate="A1" pin="GND"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="55.88" y1="111.76" x2="55.88" y2="106.68" width="0.1524" layer="91"/>
</segment>
<segment>
<wire x1="10.16" y1="124.46" x2="10.16" y2="121.92" width="0.1524" layer="91"/>
<pinref part="GND5" gate="1" pin="GND"/>
<pinref part="PWR_IN" gate="J$1" pin="GND"/>
<wire x1="13.97" y1="127" x2="13.97" y2="124.46" width="0.1524" layer="91"/>
<wire x1="13.97" y1="124.46" x2="10.16" y2="124.46" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND6" gate="1" pin="GND"/>
<wire x1="40.64" y1="109.22" x2="40.64" y2="106.68" width="0.1524" layer="91"/>
<pinref part="C65" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="GND7" gate="1" pin="GND"/>
<wire x1="73.66" y1="109.22" x2="73.66" y2="106.68" width="0.1524" layer="91"/>
<pinref part="C66" gate="G$1" pin="2"/>
</segment>
<segment>
<wire x1="200.66" y1="30.48" x2="200.66" y2="25.4" width="0.1524" layer="91"/>
<label x="203.2" y="30.48" size="1.778" layer="95"/>
<pinref part="GND11" gate="1" pin="GND"/>
<pinref part="U$8" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="45.72" x2="213.36" y2="45.72" width="0.1524" layer="91"/>
<wire x1="213.36" y1="45.72" x2="213.36" y2="30.48" width="0.1524" layer="91"/>
<wire x1="213.36" y1="30.48" x2="200.66" y2="30.48" width="0.1524" layer="91"/>
<pinref part="U$7" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="30.48" x2="213.36" y2="30.48" width="0.1524" layer="91"/>
<junction x="213.36" y="30.48"/>
<pinref part="U3" gate="A" pin="GND"/>
<junction x="200.66" y="30.48"/>
<wire x1="195.58" y1="30.48" x2="198.12" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C3" gate="G$1" pin="2"/>
<wire x1="198.12" y1="30.48" x2="200.66" y2="30.48" width="0.1524" layer="91"/>
<junction x="198.12" y="30.48"/>
</segment>
<segment>
<wire x1="200.66" y1="76.2" x2="200.66" y2="71.12" width="0.1524" layer="91"/>
<label x="203.2" y="76.2" size="1.778" layer="95"/>
<pinref part="GND10" gate="1" pin="GND"/>
<pinref part="U$6" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="91.44" x2="213.36" y2="91.44" width="0.1524" layer="91"/>
<wire x1="213.36" y1="91.44" x2="213.36" y2="76.2" width="0.1524" layer="91"/>
<wire x1="213.36" y1="76.2" x2="200.66" y2="76.2" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="76.2" x2="213.36" y2="76.2" width="0.1524" layer="91"/>
<junction x="213.36" y="76.2"/>
<pinref part="U2" gate="A" pin="GND"/>
<wire x1="195.58" y1="76.2" x2="198.12" y2="76.2" width="0.1524" layer="91"/>
<junction x="200.66" y="76.2"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="198.12" y1="76.2" x2="200.66" y2="76.2" width="0.1524" layer="91"/>
<junction x="198.12" y="76.2"/>
</segment>
<segment>
<wire x1="200.66" y1="121.92" x2="200.66" y2="116.84" width="0.1524" layer="91"/>
<label x="203.2" y="121.92" size="1.778" layer="95"/>
<pinref part="GND9" gate="1" pin="GND"/>
<pinref part="U$4" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="137.16" x2="213.36" y2="137.16" width="0.1524" layer="91"/>
<wire x1="213.36" y1="137.16" x2="213.36" y2="121.92" width="0.1524" layer="91"/>
<wire x1="213.36" y1="121.92" x2="200.66" y2="121.92" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="121.92" x2="213.36" y2="121.92" width="0.1524" layer="91"/>
<junction x="213.36" y="121.92"/>
<pinref part="U1" gate="A" pin="GND"/>
<wire x1="195.58" y1="121.92" x2="198.12" y2="121.92" width="0.1524" layer="91"/>
<junction x="200.66" y="121.92"/>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="198.12" y1="121.92" x2="200.66" y2="121.92" width="0.1524" layer="91"/>
<junction x="198.12" y="121.92"/>
</segment>
<segment>
<pinref part="U4" gate="A" pin="GND"/>
<wire x1="195.58" y1="-15.24" x2="198.12" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-15.24" x2="200.66" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="200.66" y1="-15.24" x2="200.66" y2="-20.32" width="0.1524" layer="91"/>
<label x="203.2" y="-15.24" size="1.778" layer="95"/>
<pinref part="GND12" gate="1" pin="GND"/>
<pinref part="U$10" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="0" x2="213.36" y2="0" width="0.1524" layer="91"/>
<wire x1="213.36" y1="0" x2="213.36" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-15.24" x2="200.66" y2="-15.24" width="0.1524" layer="91"/>
<junction x="200.66" y="-15.24"/>
<pinref part="U$9" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-15.24" x2="213.36" y2="-15.24" width="0.1524" layer="91"/>
<junction x="213.36" y="-15.24"/>
<pinref part="C4" gate="G$1" pin="2"/>
<junction x="198.12" y="-15.24"/>
</segment>
<segment>
<wire x1="200.66" y1="-152.4" x2="200.66" y2="-157.48" width="0.1524" layer="91"/>
<label x="203.2" y="-152.4" size="1.778" layer="95"/>
<pinref part="GND14" gate="1" pin="GND"/>
<pinref part="U$14" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-137.16" x2="213.36" y2="-137.16" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-137.16" x2="213.36" y2="-152.4" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-152.4" x2="200.66" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="U$13" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-152.4" x2="213.36" y2="-152.4" width="0.1524" layer="91"/>
<junction x="213.36" y="-152.4"/>
<pinref part="U7" gate="A" pin="GND"/>
<junction x="200.66" y="-152.4"/>
<wire x1="195.58" y1="-152.4" x2="198.12" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="C7" gate="G$1" pin="2"/>
<wire x1="198.12" y1="-152.4" x2="200.66" y2="-152.4" width="0.1524" layer="91"/>
<junction x="198.12" y="-152.4"/>
</segment>
<segment>
<wire x1="200.66" y1="-106.68" x2="200.66" y2="-111.76" width="0.1524" layer="91"/>
<label x="203.2" y="-106.68" size="1.778" layer="95"/>
<pinref part="GND13" gate="1" pin="GND"/>
<pinref part="U$12" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-91.44" x2="213.36" y2="-91.44" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-91.44" x2="213.36" y2="-106.68" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-106.68" x2="200.66" y2="-106.68" width="0.1524" layer="91"/>
<pinref part="U$11" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-106.68" x2="213.36" y2="-106.68" width="0.1524" layer="91"/>
<junction x="213.36" y="-106.68"/>
<pinref part="U6" gate="A" pin="GND"/>
<wire x1="195.58" y1="-106.68" x2="198.12" y2="-106.68" width="0.1524" layer="91"/>
<junction x="200.66" y="-106.68"/>
<pinref part="C6" gate="G$1" pin="2"/>
<wire x1="198.12" y1="-106.68" x2="200.66" y2="-106.68" width="0.1524" layer="91"/>
<junction x="198.12" y="-106.68"/>
</segment>
<segment>
<wire x1="200.66" y1="-60.96" x2="200.66" y2="-66.04" width="0.1524" layer="91"/>
<label x="203.2" y="-60.96" size="1.778" layer="95"/>
<pinref part="GND8" gate="1" pin="GND"/>
<pinref part="U$2" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-45.72" x2="213.36" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-45.72" x2="213.36" y2="-60.96" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-60.96" x2="200.66" y2="-60.96" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-60.96" x2="213.36" y2="-60.96" width="0.1524" layer="91"/>
<junction x="213.36" y="-60.96"/>
<pinref part="U5" gate="A" pin="GND"/>
<wire x1="195.58" y1="-60.96" x2="198.12" y2="-60.96" width="0.1524" layer="91"/>
<junction x="200.66" y="-60.96"/>
<pinref part="C5" gate="G$1" pin="2"/>
<wire x1="198.12" y1="-60.96" x2="200.66" y2="-60.96" width="0.1524" layer="91"/>
<junction x="198.12" y="-60.96"/>
</segment>
<segment>
<pinref part="U8" gate="A" pin="GND"/>
<wire x1="195.58" y1="-198.12" x2="198.12" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-198.12" x2="200.66" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="200.66" y1="-198.12" x2="200.66" y2="-203.2" width="0.1524" layer="91"/>
<label x="203.2" y="-198.12" size="1.778" layer="95"/>
<pinref part="GND15" gate="1" pin="GND"/>
<pinref part="U$16" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-182.88" x2="213.36" y2="-182.88" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-182.88" x2="213.36" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="213.36" y1="-198.12" x2="200.66" y2="-198.12" width="0.1524" layer="91"/>
<junction x="200.66" y="-198.12"/>
<pinref part="U$15" gate="G$1" pin="P1"/>
<wire x1="215.9" y1="-198.12" x2="213.36" y2="-198.12" width="0.1524" layer="91"/>
<junction x="213.36" y="-198.12"/>
<pinref part="C8" gate="G$1" pin="2"/>
<junction x="198.12" y="-198.12"/>
</segment>
<segment>
<wire x1="292.1" y1="30.48" x2="292.1" y2="25.4" width="0.1524" layer="91"/>
<label x="294.64" y="30.48" size="1.778" layer="95"/>
<pinref part="GND18" gate="1" pin="GND"/>
<pinref part="U$22" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="45.72" x2="304.8" y2="45.72" width="0.1524" layer="91"/>
<wire x1="304.8" y1="45.72" x2="304.8" y2="30.48" width="0.1524" layer="91"/>
<wire x1="304.8" y1="30.48" x2="292.1" y2="30.48" width="0.1524" layer="91"/>
<pinref part="U$21" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="30.48" x2="304.8" y2="30.48" width="0.1524" layer="91"/>
<junction x="304.8" y="30.48"/>
<pinref part="U11" gate="A" pin="GND"/>
<junction x="292.1" y="30.48"/>
<wire x1="287.02" y1="30.48" x2="289.56" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C11" gate="G$1" pin="2"/>
<wire x1="289.56" y1="30.48" x2="292.1" y2="30.48" width="0.1524" layer="91"/>
<junction x="289.56" y="30.48"/>
</segment>
<segment>
<wire x1="292.1" y1="76.2" x2="292.1" y2="71.12" width="0.1524" layer="91"/>
<label x="294.64" y="76.2" size="1.778" layer="95"/>
<pinref part="GND17" gate="1" pin="GND"/>
<pinref part="U$20" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="91.44" x2="304.8" y2="91.44" width="0.1524" layer="91"/>
<wire x1="304.8" y1="91.44" x2="304.8" y2="76.2" width="0.1524" layer="91"/>
<wire x1="304.8" y1="76.2" x2="292.1" y2="76.2" width="0.1524" layer="91"/>
<pinref part="U$19" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="76.2" x2="304.8" y2="76.2" width="0.1524" layer="91"/>
<junction x="304.8" y="76.2"/>
<pinref part="U10" gate="A" pin="GND"/>
<wire x1="287.02" y1="76.2" x2="289.56" y2="76.2" width="0.1524" layer="91"/>
<junction x="292.1" y="76.2"/>
<pinref part="C10" gate="G$1" pin="2"/>
<wire x1="289.56" y1="76.2" x2="292.1" y2="76.2" width="0.1524" layer="91"/>
<junction x="289.56" y="76.2"/>
</segment>
<segment>
<wire x1="292.1" y1="121.92" x2="292.1" y2="116.84" width="0.1524" layer="91"/>
<label x="294.64" y="121.92" size="1.778" layer="95"/>
<pinref part="GND16" gate="1" pin="GND"/>
<pinref part="U$18" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="137.16" x2="304.8" y2="137.16" width="0.1524" layer="91"/>
<wire x1="304.8" y1="137.16" x2="304.8" y2="121.92" width="0.1524" layer="91"/>
<wire x1="304.8" y1="121.92" x2="292.1" y2="121.92" width="0.1524" layer="91"/>
<pinref part="U$17" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="121.92" x2="304.8" y2="121.92" width="0.1524" layer="91"/>
<junction x="304.8" y="121.92"/>
<pinref part="U9" gate="A" pin="GND"/>
<wire x1="287.02" y1="121.92" x2="289.56" y2="121.92" width="0.1524" layer="91"/>
<junction x="292.1" y="121.92"/>
<pinref part="C9" gate="G$1" pin="2"/>
<wire x1="289.56" y1="121.92" x2="292.1" y2="121.92" width="0.1524" layer="91"/>
<junction x="289.56" y="121.92"/>
</segment>
<segment>
<pinref part="U12" gate="A" pin="GND"/>
<wire x1="287.02" y1="-15.24" x2="289.56" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-15.24" x2="292.1" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="292.1" y1="-15.24" x2="292.1" y2="-20.32" width="0.1524" layer="91"/>
<label x="294.64" y="-15.24" size="1.778" layer="95"/>
<pinref part="GND19" gate="1" pin="GND"/>
<pinref part="U$24" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="0" x2="304.8" y2="0" width="0.1524" layer="91"/>
<wire x1="304.8" y1="0" x2="304.8" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-15.24" x2="292.1" y2="-15.24" width="0.1524" layer="91"/>
<junction x="292.1" y="-15.24"/>
<pinref part="U$23" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-15.24" x2="304.8" y2="-15.24" width="0.1524" layer="91"/>
<junction x="304.8" y="-15.24"/>
<pinref part="C12" gate="G$1" pin="2"/>
<junction x="289.56" y="-15.24"/>
</segment>
<segment>
<wire x1="292.1" y1="-152.4" x2="292.1" y2="-157.48" width="0.1524" layer="91"/>
<label x="294.64" y="-152.4" size="1.778" layer="95"/>
<pinref part="GND22" gate="1" pin="GND"/>
<pinref part="U$30" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-137.16" x2="304.8" y2="-137.16" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-137.16" x2="304.8" y2="-152.4" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-152.4" x2="292.1" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="U$29" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-152.4" x2="304.8" y2="-152.4" width="0.1524" layer="91"/>
<junction x="304.8" y="-152.4"/>
<pinref part="U15" gate="A" pin="GND"/>
<junction x="292.1" y="-152.4"/>
<wire x1="287.02" y1="-152.4" x2="289.56" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="C15" gate="G$1" pin="2"/>
<wire x1="289.56" y1="-152.4" x2="292.1" y2="-152.4" width="0.1524" layer="91"/>
<junction x="289.56" y="-152.4"/>
</segment>
<segment>
<wire x1="292.1" y1="-106.68" x2="292.1" y2="-111.76" width="0.1524" layer="91"/>
<label x="294.64" y="-106.68" size="1.778" layer="95"/>
<pinref part="GND21" gate="1" pin="GND"/>
<pinref part="U$28" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-91.44" x2="304.8" y2="-91.44" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-91.44" x2="304.8" y2="-106.68" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-106.68" x2="292.1" y2="-106.68" width="0.1524" layer="91"/>
<pinref part="U$27" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-106.68" x2="304.8" y2="-106.68" width="0.1524" layer="91"/>
<junction x="304.8" y="-106.68"/>
<pinref part="U14" gate="A" pin="GND"/>
<wire x1="287.02" y1="-106.68" x2="289.56" y2="-106.68" width="0.1524" layer="91"/>
<junction x="292.1" y="-106.68"/>
<pinref part="C14" gate="G$1" pin="2"/>
<wire x1="289.56" y1="-106.68" x2="292.1" y2="-106.68" width="0.1524" layer="91"/>
<junction x="289.56" y="-106.68"/>
</segment>
<segment>
<wire x1="292.1" y1="-60.96" x2="292.1" y2="-66.04" width="0.1524" layer="91"/>
<label x="294.64" y="-60.96" size="1.778" layer="95"/>
<pinref part="GND20" gate="1" pin="GND"/>
<pinref part="U$26" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-45.72" x2="304.8" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-45.72" x2="304.8" y2="-60.96" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-60.96" x2="292.1" y2="-60.96" width="0.1524" layer="91"/>
<pinref part="U$25" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-60.96" x2="304.8" y2="-60.96" width="0.1524" layer="91"/>
<junction x="304.8" y="-60.96"/>
<pinref part="U13" gate="A" pin="GND"/>
<wire x1="287.02" y1="-60.96" x2="289.56" y2="-60.96" width="0.1524" layer="91"/>
<junction x="292.1" y="-60.96"/>
<pinref part="C13" gate="G$1" pin="2"/>
<wire x1="289.56" y1="-60.96" x2="292.1" y2="-60.96" width="0.1524" layer="91"/>
<junction x="289.56" y="-60.96"/>
</segment>
<segment>
<pinref part="U16" gate="A" pin="GND"/>
<wire x1="287.02" y1="-198.12" x2="289.56" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-198.12" x2="292.1" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="292.1" y1="-198.12" x2="292.1" y2="-203.2" width="0.1524" layer="91"/>
<label x="294.64" y="-198.12" size="1.778" layer="95"/>
<pinref part="GND23" gate="1" pin="GND"/>
<pinref part="U$32" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-182.88" x2="304.8" y2="-182.88" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-182.88" x2="304.8" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="304.8" y1="-198.12" x2="292.1" y2="-198.12" width="0.1524" layer="91"/>
<junction x="292.1" y="-198.12"/>
<pinref part="U$31" gate="G$1" pin="P1"/>
<wire x1="307.34" y1="-198.12" x2="304.8" y2="-198.12" width="0.1524" layer="91"/>
<junction x="304.8" y="-198.12"/>
<pinref part="C16" gate="G$1" pin="2"/>
<junction x="289.56" y="-198.12"/>
</segment>
<segment>
<wire x1="381" y1="30.48" x2="381" y2="25.4" width="0.1524" layer="91"/>
<label x="383.54" y="30.48" size="1.778" layer="95"/>
<pinref part="GND26" gate="1" pin="GND"/>
<pinref part="U$38" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="45.72" x2="393.7" y2="45.72" width="0.1524" layer="91"/>
<wire x1="393.7" y1="45.72" x2="393.7" y2="30.48" width="0.1524" layer="91"/>
<wire x1="393.7" y1="30.48" x2="381" y2="30.48" width="0.1524" layer="91"/>
<pinref part="U$37" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="30.48" x2="393.7" y2="30.48" width="0.1524" layer="91"/>
<junction x="393.7" y="30.48"/>
<pinref part="U19" gate="A" pin="GND"/>
<junction x="381" y="30.48"/>
<wire x1="375.92" y1="30.48" x2="378.46" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C19" gate="G$1" pin="2"/>
<wire x1="378.46" y1="30.48" x2="381" y2="30.48" width="0.1524" layer="91"/>
<junction x="378.46" y="30.48"/>
</segment>
<segment>
<wire x1="381" y1="76.2" x2="381" y2="71.12" width="0.1524" layer="91"/>
<label x="383.54" y="76.2" size="1.778" layer="95"/>
<pinref part="GND25" gate="1" pin="GND"/>
<pinref part="U$36" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="91.44" x2="393.7" y2="91.44" width="0.1524" layer="91"/>
<wire x1="393.7" y1="91.44" x2="393.7" y2="76.2" width="0.1524" layer="91"/>
<wire x1="393.7" y1="76.2" x2="381" y2="76.2" width="0.1524" layer="91"/>
<pinref part="U$35" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="76.2" x2="393.7" y2="76.2" width="0.1524" layer="91"/>
<junction x="393.7" y="76.2"/>
<pinref part="U18" gate="A" pin="GND"/>
<wire x1="375.92" y1="76.2" x2="378.46" y2="76.2" width="0.1524" layer="91"/>
<junction x="381" y="76.2"/>
<pinref part="C18" gate="G$1" pin="2"/>
<wire x1="378.46" y1="76.2" x2="381" y2="76.2" width="0.1524" layer="91"/>
<junction x="378.46" y="76.2"/>
</segment>
<segment>
<wire x1="381" y1="121.92" x2="381" y2="116.84" width="0.1524" layer="91"/>
<label x="383.54" y="121.92" size="1.778" layer="95"/>
<pinref part="GND24" gate="1" pin="GND"/>
<pinref part="U$34" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="137.16" x2="393.7" y2="137.16" width="0.1524" layer="91"/>
<wire x1="393.7" y1="137.16" x2="393.7" y2="121.92" width="0.1524" layer="91"/>
<wire x1="393.7" y1="121.92" x2="381" y2="121.92" width="0.1524" layer="91"/>
<pinref part="U$33" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="121.92" x2="393.7" y2="121.92" width="0.1524" layer="91"/>
<junction x="393.7" y="121.92"/>
<pinref part="U17" gate="A" pin="GND"/>
<wire x1="375.92" y1="121.92" x2="378.46" y2="121.92" width="0.1524" layer="91"/>
<junction x="381" y="121.92"/>
<pinref part="C17" gate="G$1" pin="2"/>
<wire x1="378.46" y1="121.92" x2="381" y2="121.92" width="0.1524" layer="91"/>
<junction x="378.46" y="121.92"/>
</segment>
<segment>
<pinref part="U20" gate="A" pin="GND"/>
<wire x1="375.92" y1="-15.24" x2="378.46" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-15.24" x2="381" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="381" y1="-15.24" x2="381" y2="-20.32" width="0.1524" layer="91"/>
<label x="383.54" y="-15.24" size="1.778" layer="95"/>
<pinref part="GND27" gate="1" pin="GND"/>
<pinref part="U$40" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="0" x2="393.7" y2="0" width="0.1524" layer="91"/>
<wire x1="393.7" y1="0" x2="393.7" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-15.24" x2="381" y2="-15.24" width="0.1524" layer="91"/>
<junction x="381" y="-15.24"/>
<pinref part="U$39" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-15.24" x2="393.7" y2="-15.24" width="0.1524" layer="91"/>
<junction x="393.7" y="-15.24"/>
<pinref part="C20" gate="G$1" pin="2"/>
<junction x="378.46" y="-15.24"/>
</segment>
<segment>
<wire x1="381" y1="-152.4" x2="381" y2="-157.48" width="0.1524" layer="91"/>
<label x="383.54" y="-152.4" size="1.778" layer="95"/>
<pinref part="GND30" gate="1" pin="GND"/>
<pinref part="U$46" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-137.16" x2="393.7" y2="-137.16" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-137.16" x2="393.7" y2="-152.4" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-152.4" x2="381" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="U$45" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-152.4" x2="393.7" y2="-152.4" width="0.1524" layer="91"/>
<junction x="393.7" y="-152.4"/>
<pinref part="U23" gate="A" pin="GND"/>
<junction x="381" y="-152.4"/>
<wire x1="375.92" y1="-152.4" x2="378.46" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="C23" gate="G$1" pin="2"/>
<wire x1="378.46" y1="-152.4" x2="381" y2="-152.4" width="0.1524" layer="91"/>
<junction x="378.46" y="-152.4"/>
</segment>
<segment>
<wire x1="381" y1="-106.68" x2="381" y2="-111.76" width="0.1524" layer="91"/>
<label x="383.54" y="-106.68" size="1.778" layer="95"/>
<pinref part="GND29" gate="1" pin="GND"/>
<pinref part="U$44" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-91.44" x2="393.7" y2="-91.44" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-91.44" x2="393.7" y2="-106.68" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-106.68" x2="381" y2="-106.68" width="0.1524" layer="91"/>
<pinref part="U$43" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-106.68" x2="393.7" y2="-106.68" width="0.1524" layer="91"/>
<junction x="393.7" y="-106.68"/>
<pinref part="U22" gate="A" pin="GND"/>
<wire x1="375.92" y1="-106.68" x2="378.46" y2="-106.68" width="0.1524" layer="91"/>
<junction x="381" y="-106.68"/>
<pinref part="C22" gate="G$1" pin="2"/>
<wire x1="378.46" y1="-106.68" x2="381" y2="-106.68" width="0.1524" layer="91"/>
<junction x="378.46" y="-106.68"/>
</segment>
<segment>
<wire x1="381" y1="-60.96" x2="381" y2="-66.04" width="0.1524" layer="91"/>
<label x="383.54" y="-60.96" size="1.778" layer="95"/>
<pinref part="GND28" gate="1" pin="GND"/>
<pinref part="U$42" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-45.72" x2="393.7" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-45.72" x2="393.7" y2="-60.96" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-60.96" x2="381" y2="-60.96" width="0.1524" layer="91"/>
<pinref part="U$41" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-60.96" x2="393.7" y2="-60.96" width="0.1524" layer="91"/>
<junction x="393.7" y="-60.96"/>
<pinref part="U21" gate="A" pin="GND"/>
<wire x1="375.92" y1="-60.96" x2="378.46" y2="-60.96" width="0.1524" layer="91"/>
<junction x="381" y="-60.96"/>
<pinref part="C21" gate="G$1" pin="2"/>
<wire x1="378.46" y1="-60.96" x2="381" y2="-60.96" width="0.1524" layer="91"/>
<junction x="378.46" y="-60.96"/>
</segment>
<segment>
<pinref part="U24" gate="A" pin="GND"/>
<wire x1="375.92" y1="-198.12" x2="378.46" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-198.12" x2="381" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="381" y1="-198.12" x2="381" y2="-203.2" width="0.1524" layer="91"/>
<label x="383.54" y="-198.12" size="1.778" layer="95"/>
<pinref part="GND31" gate="1" pin="GND"/>
<pinref part="U$48" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-182.88" x2="393.7" y2="-182.88" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-182.88" x2="393.7" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="393.7" y1="-198.12" x2="381" y2="-198.12" width="0.1524" layer="91"/>
<junction x="381" y="-198.12"/>
<pinref part="U$47" gate="G$1" pin="P1"/>
<wire x1="396.24" y1="-198.12" x2="393.7" y2="-198.12" width="0.1524" layer="91"/>
<junction x="393.7" y="-198.12"/>
<pinref part="C24" gate="G$1" pin="2"/>
<junction x="378.46" y="-198.12"/>
</segment>
<segment>
<wire x1="469.9" y1="30.48" x2="469.9" y2="25.4" width="0.1524" layer="91"/>
<label x="472.44" y="30.48" size="1.778" layer="95"/>
<pinref part="GND34" gate="1" pin="GND"/>
<pinref part="U$54" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="45.72" x2="482.6" y2="45.72" width="0.1524" layer="91"/>
<wire x1="482.6" y1="45.72" x2="482.6" y2="30.48" width="0.1524" layer="91"/>
<wire x1="482.6" y1="30.48" x2="469.9" y2="30.48" width="0.1524" layer="91"/>
<pinref part="U$53" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="30.48" x2="482.6" y2="30.48" width="0.1524" layer="91"/>
<junction x="482.6" y="30.48"/>
<pinref part="U27" gate="A" pin="GND"/>
<junction x="469.9" y="30.48"/>
<wire x1="464.82" y1="30.48" x2="467.36" y2="30.48" width="0.1524" layer="91"/>
<pinref part="C27" gate="G$1" pin="2"/>
<wire x1="467.36" y1="30.48" x2="469.9" y2="30.48" width="0.1524" layer="91"/>
<junction x="467.36" y="30.48"/>
</segment>
<segment>
<wire x1="469.9" y1="76.2" x2="469.9" y2="71.12" width="0.1524" layer="91"/>
<label x="472.44" y="76.2" size="1.778" layer="95"/>
<pinref part="GND33" gate="1" pin="GND"/>
<pinref part="U$52" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="91.44" x2="482.6" y2="91.44" width="0.1524" layer="91"/>
<wire x1="482.6" y1="91.44" x2="482.6" y2="76.2" width="0.1524" layer="91"/>
<wire x1="482.6" y1="76.2" x2="469.9" y2="76.2" width="0.1524" layer="91"/>
<pinref part="U$51" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="76.2" x2="482.6" y2="76.2" width="0.1524" layer="91"/>
<junction x="482.6" y="76.2"/>
<pinref part="U26" gate="A" pin="GND"/>
<wire x1="464.82" y1="76.2" x2="467.36" y2="76.2" width="0.1524" layer="91"/>
<junction x="469.9" y="76.2"/>
<pinref part="C26" gate="G$1" pin="2"/>
<wire x1="467.36" y1="76.2" x2="469.9" y2="76.2" width="0.1524" layer="91"/>
<junction x="467.36" y="76.2"/>
</segment>
<segment>
<wire x1="469.9" y1="121.92" x2="469.9" y2="116.84" width="0.1524" layer="91"/>
<label x="472.44" y="121.92" size="1.778" layer="95"/>
<pinref part="GND32" gate="1" pin="GND"/>
<pinref part="U$50" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="137.16" x2="482.6" y2="137.16" width="0.1524" layer="91"/>
<wire x1="482.6" y1="137.16" x2="482.6" y2="121.92" width="0.1524" layer="91"/>
<wire x1="482.6" y1="121.92" x2="469.9" y2="121.92" width="0.1524" layer="91"/>
<pinref part="U$49" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="121.92" x2="482.6" y2="121.92" width="0.1524" layer="91"/>
<junction x="482.6" y="121.92"/>
<pinref part="U25" gate="A" pin="GND"/>
<wire x1="464.82" y1="121.92" x2="467.36" y2="121.92" width="0.1524" layer="91"/>
<junction x="469.9" y="121.92"/>
<pinref part="C25" gate="G$1" pin="2"/>
<wire x1="467.36" y1="121.92" x2="469.9" y2="121.92" width="0.1524" layer="91"/>
<junction x="467.36" y="121.92"/>
</segment>
<segment>
<pinref part="U28" gate="A" pin="GND"/>
<wire x1="464.82" y1="-15.24" x2="467.36" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-15.24" x2="469.9" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="469.9" y1="-15.24" x2="469.9" y2="-20.32" width="0.1524" layer="91"/>
<label x="472.44" y="-15.24" size="1.778" layer="95"/>
<pinref part="GND35" gate="1" pin="GND"/>
<pinref part="U$56" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="0" x2="482.6" y2="0" width="0.1524" layer="91"/>
<wire x1="482.6" y1="0" x2="482.6" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-15.24" x2="469.9" y2="-15.24" width="0.1524" layer="91"/>
<junction x="469.9" y="-15.24"/>
<pinref part="U$55" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-15.24" x2="482.6" y2="-15.24" width="0.1524" layer="91"/>
<junction x="482.6" y="-15.24"/>
<pinref part="C28" gate="G$1" pin="2"/>
<junction x="467.36" y="-15.24"/>
</segment>
<segment>
<wire x1="469.9" y1="-152.4" x2="469.9" y2="-157.48" width="0.1524" layer="91"/>
<label x="472.44" y="-152.4" size="1.778" layer="95"/>
<pinref part="GND38" gate="1" pin="GND"/>
<pinref part="U$62" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-137.16" x2="482.6" y2="-137.16" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-137.16" x2="482.6" y2="-152.4" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-152.4" x2="469.9" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="U$61" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-152.4" x2="482.6" y2="-152.4" width="0.1524" layer="91"/>
<junction x="482.6" y="-152.4"/>
<pinref part="U31" gate="A" pin="GND"/>
<junction x="469.9" y="-152.4"/>
<wire x1="464.82" y1="-152.4" x2="467.36" y2="-152.4" width="0.1524" layer="91"/>
<pinref part="C31" gate="G$1" pin="2"/>
<wire x1="467.36" y1="-152.4" x2="469.9" y2="-152.4" width="0.1524" layer="91"/>
<junction x="467.36" y="-152.4"/>
</segment>
<segment>
<wire x1="469.9" y1="-106.68" x2="469.9" y2="-111.76" width="0.1524" layer="91"/>
<label x="472.44" y="-106.68" size="1.778" layer="95"/>
<pinref part="GND37" gate="1" pin="GND"/>
<pinref part="U$60" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-91.44" x2="482.6" y2="-91.44" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-91.44" x2="482.6" y2="-106.68" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-106.68" x2="469.9" y2="-106.68" width="0.1524" layer="91"/>
<pinref part="U$59" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-106.68" x2="482.6" y2="-106.68" width="0.1524" layer="91"/>
<junction x="482.6" y="-106.68"/>
<pinref part="U30" gate="A" pin="GND"/>
<wire x1="464.82" y1="-106.68" x2="467.36" y2="-106.68" width="0.1524" layer="91"/>
<junction x="469.9" y="-106.68"/>
<pinref part="C30" gate="G$1" pin="2"/>
<wire x1="467.36" y1="-106.68" x2="469.9" y2="-106.68" width="0.1524" layer="91"/>
<junction x="467.36" y="-106.68"/>
</segment>
<segment>
<wire x1="469.9" y1="-60.96" x2="469.9" y2="-66.04" width="0.1524" layer="91"/>
<label x="472.44" y="-60.96" size="1.778" layer="95"/>
<pinref part="GND36" gate="1" pin="GND"/>
<pinref part="U$58" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-45.72" x2="482.6" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-45.72" x2="482.6" y2="-60.96" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-60.96" x2="469.9" y2="-60.96" width="0.1524" layer="91"/>
<pinref part="U$57" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-60.96" x2="482.6" y2="-60.96" width="0.1524" layer="91"/>
<junction x="482.6" y="-60.96"/>
<pinref part="U29" gate="A" pin="GND"/>
<wire x1="464.82" y1="-60.96" x2="467.36" y2="-60.96" width="0.1524" layer="91"/>
<junction x="469.9" y="-60.96"/>
<pinref part="C29" gate="G$1" pin="2"/>
<wire x1="467.36" y1="-60.96" x2="469.9" y2="-60.96" width="0.1524" layer="91"/>
<junction x="467.36" y="-60.96"/>
</segment>
<segment>
<pinref part="U32" gate="A" pin="GND"/>
<wire x1="464.82" y1="-198.12" x2="467.36" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-198.12" x2="469.9" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="469.9" y1="-198.12" x2="469.9" y2="-203.2" width="0.1524" layer="91"/>
<label x="472.44" y="-198.12" size="1.778" layer="95"/>
<pinref part="GND39" gate="1" pin="GND"/>
<pinref part="U$64" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-182.88" x2="482.6" y2="-182.88" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-182.88" x2="482.6" y2="-198.12" width="0.1524" layer="91"/>
<wire x1="482.6" y1="-198.12" x2="469.9" y2="-198.12" width="0.1524" layer="91"/>
<junction x="469.9" y="-198.12"/>
<pinref part="U$63" gate="G$1" pin="P1"/>
<wire x1="485.14" y1="-198.12" x2="482.6" y2="-198.12" width="0.1524" layer="91"/>
<junction x="482.6" y="-198.12"/>
<pinref part="C32" gate="G$1" pin="2"/>
<junction x="467.36" y="-198.12"/>
</segment>
<segment>
<pinref part="GND40" gate="1" pin="GND"/>
<pinref part="C33" gate="G$1" pin="2"/>
<wire x1="228.6" y1="116.84" x2="228.6" y2="119.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND41" gate="1" pin="GND"/>
<pinref part="C34" gate="G$1" pin="2"/>
<wire x1="228.6" y1="71.12" x2="228.6" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND42" gate="1" pin="GND"/>
<pinref part="C35" gate="G$1" pin="2"/>
<wire x1="228.6" y1="25.4" x2="228.6" y2="27.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND43" gate="1" pin="GND"/>
<pinref part="C36" gate="G$1" pin="2"/>
<wire x1="228.6" y1="-20.32" x2="228.6" y2="-17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND44" gate="1" pin="GND"/>
<pinref part="C37" gate="G$1" pin="2"/>
<wire x1="228.6" y1="-66.04" x2="228.6" y2="-63.5" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND45" gate="1" pin="GND"/>
<pinref part="C38" gate="G$1" pin="2"/>
<wire x1="228.6" y1="-111.76" x2="228.6" y2="-109.22" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND46" gate="1" pin="GND"/>
<pinref part="C39" gate="G$1" pin="2"/>
<wire x1="228.6" y1="-157.48" x2="228.6" y2="-154.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND47" gate="1" pin="GND"/>
<pinref part="C40" gate="G$1" pin="2"/>
<wire x1="228.6" y1="-203.2" x2="228.6" y2="-200.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND48" gate="1" pin="GND"/>
<pinref part="C41" gate="G$1" pin="2"/>
<wire x1="320.04" y1="116.84" x2="320.04" y2="119.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND49" gate="1" pin="GND"/>
<pinref part="C42" gate="G$1" pin="2"/>
<wire x1="320.04" y1="71.12" x2="320.04" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND50" gate="1" pin="GND"/>
<pinref part="C43" gate="G$1" pin="2"/>
<wire x1="320.04" y1="25.4" x2="320.04" y2="27.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND51" gate="1" pin="GND"/>
<pinref part="C44" gate="G$1" pin="2"/>
<wire x1="320.04" y1="-20.32" x2="320.04" y2="-17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND52" gate="1" pin="GND"/>
<pinref part="C45" gate="G$1" pin="2"/>
<wire x1="320.04" y1="-66.04" x2="320.04" y2="-63.5" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND53" gate="1" pin="GND"/>
<pinref part="C46" gate="G$1" pin="2"/>
<wire x1="320.04" y1="-111.76" x2="320.04" y2="-109.22" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND54" gate="1" pin="GND"/>
<pinref part="C47" gate="G$1" pin="2"/>
<wire x1="320.04" y1="-157.48" x2="320.04" y2="-154.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND55" gate="1" pin="GND"/>
<pinref part="C48" gate="G$1" pin="2"/>
<wire x1="320.04" y1="-203.2" x2="320.04" y2="-200.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND56" gate="1" pin="GND"/>
<pinref part="C49" gate="G$1" pin="2"/>
<wire x1="408.94" y1="116.84" x2="408.94" y2="119.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND57" gate="1" pin="GND"/>
<pinref part="C50" gate="G$1" pin="2"/>
<wire x1="408.94" y1="71.12" x2="408.94" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND58" gate="1" pin="GND"/>
<pinref part="C51" gate="G$1" pin="2"/>
<wire x1="408.94" y1="25.4" x2="408.94" y2="27.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND59" gate="1" pin="GND"/>
<pinref part="C52" gate="G$1" pin="2"/>
<wire x1="408.94" y1="-20.32" x2="408.94" y2="-17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND60" gate="1" pin="GND"/>
<pinref part="C53" gate="G$1" pin="2"/>
<wire x1="408.94" y1="-66.04" x2="408.94" y2="-63.5" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND61" gate="1" pin="GND"/>
<pinref part="C54" gate="G$1" pin="2"/>
<wire x1="408.94" y1="-111.76" x2="408.94" y2="-109.22" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND62" gate="1" pin="GND"/>
<pinref part="C55" gate="G$1" pin="2"/>
<wire x1="408.94" y1="-157.48" x2="408.94" y2="-154.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND63" gate="1" pin="GND"/>
<pinref part="C56" gate="G$1" pin="2"/>
<wire x1="408.94" y1="-203.2" x2="408.94" y2="-200.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND64" gate="1" pin="GND"/>
<pinref part="C57" gate="G$1" pin="2"/>
<wire x1="497.84" y1="116.84" x2="497.84" y2="119.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND65" gate="1" pin="GND"/>
<pinref part="C58" gate="G$1" pin="2"/>
<wire x1="497.84" y1="71.12" x2="497.84" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND66" gate="1" pin="GND"/>
<pinref part="C59" gate="G$1" pin="2"/>
<wire x1="497.84" y1="25.4" x2="497.84" y2="27.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND67" gate="1" pin="GND"/>
<pinref part="C60" gate="G$1" pin="2"/>
<wire x1="497.84" y1="-20.32" x2="497.84" y2="-17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND68" gate="1" pin="GND"/>
<pinref part="C61" gate="G$1" pin="2"/>
<wire x1="497.84" y1="-66.04" x2="497.84" y2="-63.5" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND69" gate="1" pin="GND"/>
<pinref part="C62" gate="G$1" pin="2"/>
<wire x1="497.84" y1="-111.76" x2="497.84" y2="-109.22" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND70" gate="1" pin="GND"/>
<pinref part="C63" gate="G$1" pin="2"/>
<wire x1="497.84" y1="-157.48" x2="497.84" y2="-154.94" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND71" gate="1" pin="GND"/>
<pinref part="C64" gate="G$1" pin="2"/>
<wire x1="497.84" y1="-203.2" x2="497.84" y2="-200.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="RESET" gate="1" pin="S"/>
<wire x1="0" y1="104.14" x2="5.08" y2="104.14" width="0.1524" layer="91"/>
<wire x1="5.08" y1="104.14" x2="12.7" y2="104.14" width="0.1524" layer="91"/>
<wire x1="12.7" y1="104.14" x2="12.7" y2="99.06" width="0.1524" layer="91"/>
<pinref part="GND72" gate="1" pin="GND"/>
<pinref part="RESET" gate="1" pin="S1"/>
<wire x1="0" y1="101.6" x2="5.08" y2="101.6" width="0.1524" layer="91"/>
<wire x1="5.08" y1="101.6" x2="5.08" y2="104.14" width="0.1524" layer="91"/>
<junction x="5.08" y="104.14"/>
</segment>
</net>
<net name="AREF" class="0">
<segment>
<wire x1="96.52" y1="58.42" x2="109.22" y2="58.42" width="0.1524" layer="91"/>
<label x="101.6" y="58.42" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="8"/>
<pinref part="SVO3" gate="1" pin="8"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<wire x1="96.52" y1="60.96" x2="109.22" y2="60.96" width="0.1524" layer="91"/>
<label x="101.6" y="60.96" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="9"/>
<pinref part="SVO3" gate="1" pin="9"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<wire x1="96.52" y1="63.5" x2="109.22" y2="63.5" width="0.1524" layer="91"/>
<label x="101.6" y="63.5" size="1.778" layer="95"/>
<pinref part="SVI3" gate="1" pin="10"/>
<pinref part="SVO3" gate="1" pin="10"/>
</segment>
</net>
<net name="PA14" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="8"/>
<wire x1="96.52" y1="2.54" x2="109.22" y2="2.54" width="0.1524" layer="91"/>
<label x="101.6" y="2.54" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="8"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="-48.26" x2="144.78" y2="-48.26" width="0.1524" layer="91"/>
<label x="147.32" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA15" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="7"/>
<wire x1="96.52" y1="0" x2="109.22" y2="0" width="0.1524" layer="91"/>
<label x="101.6" y="0" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="7"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="-50.8" x2="144.78" y2="-50.8" width="0.1524" layer="91"/>
<label x="147.32" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA16" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="6"/>
<wire x1="96.52" y1="-2.54" x2="109.22" y2="-2.54" width="0.1524" layer="91"/>
<label x="101.6" y="-2.54" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="6"/>
</segment>
<segment>
<pinref part="U6" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="-93.98" x2="144.78" y2="-93.98" width="0.1524" layer="91"/>
<label x="147.32" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA17" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="5"/>
<wire x1="96.52" y1="-5.08" x2="109.22" y2="-5.08" width="0.1524" layer="91"/>
<label x="101.6" y="-5.08" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="5"/>
</segment>
<segment>
<pinref part="U6" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="-96.52" x2="144.78" y2="-96.52" width="0.1524" layer="91"/>
<label x="147.32" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA18" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="4"/>
<wire x1="96.52" y1="-7.62" x2="109.22" y2="-7.62" width="0.1524" layer="91"/>
<label x="101.6" y="-7.62" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="4"/>
</segment>
<segment>
<pinref part="U7" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="-139.7" x2="144.78" y2="-139.7" width="0.1524" layer="91"/>
<label x="147.32" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA19" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="3"/>
<wire x1="96.52" y1="-10.16" x2="109.22" y2="-10.16" width="0.1524" layer="91"/>
<label x="101.6" y="-10.16" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="3"/>
</segment>
<segment>
<pinref part="U7" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="-142.24" x2="144.78" y2="-142.24" width="0.1524" layer="91"/>
<label x="147.32" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA20" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="2"/>
<wire x1="96.52" y1="-12.7" x2="109.22" y2="-12.7" width="0.1524" layer="91"/>
<label x="101.6" y="-12.7" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="2"/>
</segment>
<segment>
<pinref part="U8" gate="A" pin="IN_A"/>
<wire x1="160.02" y1="-185.42" x2="144.78" y2="-185.42" width="0.1524" layer="91"/>
<label x="147.32" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA21" class="0">
<segment>
<pinref part="SVI1" gate="1" pin="1"/>
<wire x1="96.52" y1="-15.24" x2="109.22" y2="-15.24" width="0.1524" layer="91"/>
<label x="101.6" y="-15.24" size="1.778" layer="95"/>
<pinref part="SVO1" gate="1" pin="1"/>
</segment>
<segment>
<pinref part="U8" gate="A" pin="IN_B"/>
<wire x1="160.02" y1="-187.96" x2="144.78" y2="-187.96" width="0.1524" layer="91"/>
<label x="147.32" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA53" class="0">
<segment>
<wire x1="43.18" y1="-48.26" x2="43.18" y2="-35.56" width="0.1524" layer="91"/>
<label x="43.18" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="34"/>
</segment>
<segment>
<pinref part="U24" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="-187.96" x2="325.12" y2="-187.96" width="0.1524" layer="91"/>
<label x="327.66" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA22" class="0">
<segment>
<label x="81.28" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="3"/>
<wire x1="81.28" y1="-20.32" x2="81.28" y2="4.9276" width="0.1524" layer="91"/>
<wire x1="81.28" y1="7.62" x2="81.28" y2="4.9276" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U9" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="132.08" x2="236.22" y2="132.08" width="0.1524" layer="91"/>
<label x="238.76" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA23" class="0">
<segment>
<wire x1="81.28" y1="-48.26" x2="81.28" y2="-35.56" width="0.1524" layer="91"/>
<label x="81.28" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="4"/>
</segment>
<segment>
<pinref part="U10" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="88.9" x2="236.22" y2="88.9" width="0.1524" layer="91"/>
<label x="238.76" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA24" class="0">
<segment>
<label x="78.74" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="5"/>
<wire x1="78.74" y1="-20.32" x2="78.74" y2="4.9276" width="0.1524" layer="91"/>
<wire x1="78.74" y1="7.62" x2="78.74" y2="4.9276" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U9" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="134.62" x2="236.22" y2="134.62" width="0.1524" layer="91"/>
<label x="238.76" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA25" class="0">
<segment>
<wire x1="78.74" y1="-48.26" x2="78.74" y2="-35.56" width="0.1524" layer="91"/>
<label x="78.74" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="6"/>
</segment>
<segment>
<pinref part="U10" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="86.36" x2="236.22" y2="86.36" width="0.1524" layer="91"/>
<label x="238.76" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA26" class="0">
<segment>
<label x="76.2" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="7"/>
<wire x1="76.2" y1="-20.32" x2="76.2" y2="4.9276" width="0.1524" layer="91"/>
<wire x1="76.2" y1="7.62" x2="76.2" y2="4.9276" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U11" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="40.64" x2="236.22" y2="40.64" width="0.1524" layer="91"/>
<label x="238.76" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA27" class="0">
<segment>
<wire x1="76.2" y1="-48.26" x2="76.2" y2="-35.56" width="0.1524" layer="91"/>
<label x="76.2" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="8"/>
</segment>
<segment>
<pinref part="U12" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="-2.54" x2="236.22" y2="-2.54" width="0.1524" layer="91"/>
<label x="238.76" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA28" class="0">
<segment>
<label x="73.66" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="9"/>
<wire x1="73.66" y1="-20.32" x2="73.66" y2="4.9276" width="0.1524" layer="91"/>
<wire x1="73.66" y1="7.62" x2="73.66" y2="4.9276" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U11" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="43.18" x2="236.22" y2="43.18" width="0.1524" layer="91"/>
<label x="238.76" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA29" class="0">
<segment>
<wire x1="73.66" y1="-48.26" x2="73.66" y2="-35.56" width="0.1524" layer="91"/>
<label x="73.66" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="10"/>
</segment>
<segment>
<pinref part="U12" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="-5.08" x2="236.22" y2="-5.08" width="0.1524" layer="91"/>
<label x="238.76" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA30" class="0">
<segment>
<label x="71.12" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="11"/>
<wire x1="71.12" y1="-20.32" x2="71.12" y2="4.9276" width="0.1524" layer="91"/>
<wire x1="71.12" y1="7.62" x2="71.12" y2="4.9276" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U13" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="-50.8" x2="236.22" y2="-50.8" width="0.1524" layer="91"/>
<label x="238.76" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA31" class="0">
<segment>
<wire x1="71.12" y1="-48.26" x2="71.12" y2="-35.56" width="0.1524" layer="91"/>
<label x="71.12" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="12"/>
</segment>
<segment>
<pinref part="U14" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="-93.98" x2="236.22" y2="-93.98" width="0.1524" layer="91"/>
<label x="238.76" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA32" class="0">
<segment>
<wire x1="68.58" y1="-20.32" x2="68.58" y2="-2.54" width="0.1524" layer="91"/>
<label x="68.58" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="13"/>
</segment>
<segment>
<pinref part="U13" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="-48.26" x2="236.22" y2="-48.26" width="0.1524" layer="91"/>
<label x="238.76" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA33" class="0">
<segment>
<wire x1="68.58" y1="-48.26" x2="68.58" y2="-35.56" width="0.1524" layer="91"/>
<label x="68.58" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="14"/>
</segment>
<segment>
<pinref part="U14" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="-96.52" x2="236.22" y2="-96.52" width="0.1524" layer="91"/>
<label x="238.76" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA34" class="0">
<segment>
<wire x1="66.04" y1="-20.32" x2="66.04" y2="-2.54" width="0.1524" layer="91"/>
<label x="66.04" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="15"/>
</segment>
<segment>
<pinref part="U15" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="-142.24" x2="236.22" y2="-142.24" width="0.1524" layer="91"/>
<label x="238.76" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA35" class="0">
<segment>
<wire x1="66.04" y1="-48.26" x2="66.04" y2="-35.56" width="0.1524" layer="91"/>
<label x="66.04" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="16"/>
</segment>
<segment>
<pinref part="U16" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="-185.42" x2="236.22" y2="-185.42" width="0.1524" layer="91"/>
<label x="238.76" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA36" class="0">
<segment>
<wire x1="63.5" y1="-20.32" x2="63.5" y2="-2.54" width="0.1524" layer="91"/>
<label x="63.5" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="17"/>
</segment>
<segment>
<pinref part="U15" gate="A" pin="IN_A"/>
<wire x1="251.46" y1="-139.7" x2="236.22" y2="-139.7" width="0.1524" layer="91"/>
<label x="238.76" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA37" class="0">
<segment>
<wire x1="63.5" y1="-48.26" x2="63.5" y2="-35.56" width="0.1524" layer="91"/>
<label x="63.5" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="18"/>
</segment>
<segment>
<pinref part="U16" gate="A" pin="IN_B"/>
<wire x1="251.46" y1="-187.96" x2="236.22" y2="-187.96" width="0.1524" layer="91"/>
<label x="238.76" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA38" class="0">
<segment>
<wire x1="60.96" y1="-20.32" x2="60.96" y2="-2.54" width="0.1524" layer="91"/>
<label x="60.96" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="19"/>
</segment>
<segment>
<pinref part="U17" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="132.08" x2="325.12" y2="132.08" width="0.1524" layer="91"/>
<label x="327.66" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA39" class="0">
<segment>
<wire x1="60.96" y1="-48.26" x2="60.96" y2="-35.56" width="0.1524" layer="91"/>
<label x="60.96" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="20"/>
</segment>
<segment>
<pinref part="U18" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="88.9" x2="325.12" y2="88.9" width="0.1524" layer="91"/>
<label x="327.66" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA40" class="0">
<segment>
<wire x1="58.42" y1="-20.32" x2="58.42" y2="-2.54" width="0.1524" layer="91"/>
<label x="58.42" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="21"/>
</segment>
<segment>
<pinref part="U17" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="134.62" x2="325.12" y2="134.62" width="0.1524" layer="91"/>
<label x="327.66" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA41" class="0">
<segment>
<wire x1="58.42" y1="-48.26" x2="58.42" y2="-35.56" width="0.1524" layer="91"/>
<label x="58.42" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="22"/>
</segment>
<segment>
<pinref part="U18" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="86.36" x2="325.12" y2="86.36" width="0.1524" layer="91"/>
<label x="327.66" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA42" class="0">
<segment>
<wire x1="55.88" y1="-20.32" x2="55.88" y2="-2.54" width="0.1524" layer="91"/>
<label x="55.88" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="23"/>
</segment>
<segment>
<pinref part="U19" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="40.64" x2="325.12" y2="40.64" width="0.1524" layer="91"/>
<label x="327.66" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA43" class="0">
<segment>
<wire x1="55.88" y1="-48.26" x2="55.88" y2="-35.56" width="0.1524" layer="91"/>
<label x="55.88" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="24"/>
</segment>
<segment>
<pinref part="U20" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="-2.54" x2="325.12" y2="-2.54" width="0.1524" layer="91"/>
<label x="327.66" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA44" class="0">
<segment>
<wire x1="53.34" y1="-20.32" x2="53.34" y2="-2.54" width="0.1524" layer="91"/>
<label x="53.34" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="25"/>
</segment>
<segment>
<pinref part="U19" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="43.18" x2="325.12" y2="43.18" width="0.1524" layer="91"/>
<label x="327.66" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA45" class="0">
<segment>
<wire x1="53.34" y1="-48.26" x2="53.34" y2="-35.56" width="0.1524" layer="91"/>
<label x="53.34" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="26"/>
</segment>
<segment>
<pinref part="U20" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="-5.08" x2="325.12" y2="-5.08" width="0.1524" layer="91"/>
<label x="327.66" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA46" class="0">
<segment>
<wire x1="50.8" y1="-20.32" x2="50.8" y2="-2.54" width="0.1524" layer="91"/>
<label x="50.8" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="27"/>
</segment>
<segment>
<pinref part="U21" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="-50.8" x2="325.12" y2="-50.8" width="0.1524" layer="91"/>
<label x="327.66" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA47" class="0">
<segment>
<wire x1="50.8" y1="-48.26" x2="50.8" y2="-35.56" width="0.1524" layer="91"/>
<label x="50.8" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="28"/>
</segment>
<segment>
<pinref part="U22" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="-93.98" x2="325.12" y2="-93.98" width="0.1524" layer="91"/>
<label x="327.66" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA48" class="0">
<segment>
<wire x1="48.26" y1="-20.32" x2="48.26" y2="-2.54" width="0.1524" layer="91"/>
<label x="48.26" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="29"/>
</segment>
<segment>
<pinref part="U21" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="-48.26" x2="325.12" y2="-48.26" width="0.1524" layer="91"/>
<label x="327.66" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA49" class="0">
<segment>
<wire x1="48.26" y1="-48.26" x2="48.26" y2="-35.56" width="0.1524" layer="91"/>
<label x="48.26" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="30"/>
</segment>
<segment>
<pinref part="U22" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="-96.52" x2="325.12" y2="-96.52" width="0.1524" layer="91"/>
<label x="327.66" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA50" class="0">
<segment>
<wire x1="45.72" y1="-20.32" x2="45.72" y2="-2.54" width="0.1524" layer="91"/>
<label x="45.72" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="31"/>
</segment>
<segment>
<pinref part="U23" gate="A" pin="IN_B"/>
<wire x1="340.36" y1="-142.24" x2="325.12" y2="-142.24" width="0.1524" layer="91"/>
<label x="327.66" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA51" class="0">
<segment>
<wire x1="45.72" y1="-48.26" x2="45.72" y2="-35.56" width="0.1524" layer="91"/>
<label x="45.72" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="32"/>
</segment>
<segment>
<pinref part="U24" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="-185.42" x2="325.12" y2="-185.42" width="0.1524" layer="91"/>
<label x="327.66" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA52" class="0">
<segment>
<wire x1="43.18" y1="-20.32" x2="43.18" y2="-2.54" width="0.1524" layer="91"/>
<label x="43.18" y="-15.24" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="33"/>
</segment>
<segment>
<pinref part="U23" gate="A" pin="IN_A"/>
<wire x1="340.36" y1="-139.7" x2="325.12" y2="-139.7" width="0.1524" layer="91"/>
<label x="327.66" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA54" class="0">
<segment>
<wire x1="15.24" y1="30.48" x2="27.94" y2="30.48" width="0.1524" layer="91"/>
<label x="17.78" y="30.48" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="8"/>
<pinref part="SVI5" gate="1" pin="8"/>
</segment>
<segment>
<pinref part="U25" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="134.62" x2="414.02" y2="134.62" width="0.1524" layer="91"/>
<label x="416.56" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA55" class="0">
<segment>
<wire x1="15.24" y1="27.94" x2="27.94" y2="27.94" width="0.1524" layer="91"/>
<label x="17.78" y="27.94" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="7"/>
<pinref part="SVI5" gate="1" pin="7"/>
</segment>
<segment>
<pinref part="U25" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="132.08" x2="414.02" y2="132.08" width="0.1524" layer="91"/>
<label x="416.56" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA69" class="0">
<segment>
<wire x1="15.24" y1="-15.24" x2="27.94" y2="-15.24" width="0.1524" layer="91"/>
<label x="17.78" y="-15.24" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="1"/>
<pinref part="SVI6" gate="1" pin="1"/>
</segment>
<segment>
<pinref part="U32" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="-185.42" x2="414.02" y2="-185.42" width="0.1524" layer="91"/>
<label x="416.56" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA56" class="0">
<segment>
<wire x1="15.24" y1="25.4" x2="27.94" y2="25.4" width="0.1524" layer="91"/>
<label x="17.78" y="25.4" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="6"/>
<pinref part="SVI5" gate="1" pin="6"/>
</segment>
<segment>
<pinref part="U26" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="88.9" x2="414.02" y2="88.9" width="0.1524" layer="91"/>
<label x="416.56" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA57" class="0">
<segment>
<wire x1="15.24" y1="22.86" x2="27.94" y2="22.86" width="0.1524" layer="91"/>
<label x="17.78" y="22.86" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="5"/>
<pinref part="SVI5" gate="1" pin="5"/>
</segment>
<segment>
<pinref part="U26" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="86.36" x2="414.02" y2="86.36" width="0.1524" layer="91"/>
<label x="416.56" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA58" class="0">
<segment>
<wire x1="15.24" y1="20.32" x2="27.94" y2="20.32" width="0.1524" layer="91"/>
<label x="17.78" y="20.32" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="4"/>
<pinref part="SVI5" gate="1" pin="4"/>
</segment>
<segment>
<pinref part="U27" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="43.18" x2="414.02" y2="43.18" width="0.1524" layer="91"/>
<label x="416.56" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA59" class="0">
<segment>
<wire x1="15.24" y1="17.78" x2="27.94" y2="17.78" width="0.1524" layer="91"/>
<label x="17.78" y="17.78" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="3"/>
<pinref part="SVI5" gate="1" pin="3"/>
</segment>
<segment>
<pinref part="U27" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="40.64" x2="414.02" y2="40.64" width="0.1524" layer="91"/>
<label x="416.56" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA60" class="0">
<segment>
<wire x1="15.24" y1="15.24" x2="27.94" y2="15.24" width="0.1524" layer="91"/>
<label x="17.78" y="15.24" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="2"/>
<pinref part="SVI5" gate="1" pin="2"/>
</segment>
<segment>
<pinref part="U28" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="-2.54" x2="414.02" y2="-2.54" width="0.1524" layer="91"/>
<label x="416.56" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA61" class="0">
<segment>
<wire x1="15.24" y1="12.7" x2="27.94" y2="12.7" width="0.1524" layer="91"/>
<label x="17.78" y="12.7" size="1.778" layer="95"/>
<pinref part="SVO5" gate="1" pin="1"/>
<pinref part="SVI5" gate="1" pin="1"/>
</segment>
<segment>
<pinref part="U28" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="-5.08" x2="414.02" y2="-5.08" width="0.1524" layer="91"/>
<label x="416.56" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA62" class="0">
<segment>
<wire x1="15.24" y1="2.54" x2="27.94" y2="2.54" width="0.1524" layer="91"/>
<label x="17.78" y="2.54" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="8"/>
<pinref part="SVI6" gate="1" pin="8"/>
</segment>
<segment>
<pinref part="U29" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="-48.26" x2="414.02" y2="-48.26" width="0.1524" layer="91"/>
<label x="416.56" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA63" class="0">
<segment>
<wire x1="15.24" y1="0" x2="27.94" y2="0" width="0.1524" layer="91"/>
<label x="17.78" y="0" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="7"/>
<pinref part="SVI6" gate="1" pin="7"/>
</segment>
<segment>
<pinref part="U29" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="-50.8" x2="414.02" y2="-50.8" width="0.1524" layer="91"/>
<label x="416.56" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA64" class="0">
<segment>
<wire x1="15.24" y1="-2.54" x2="27.94" y2="-2.54" width="0.1524" layer="91"/>
<label x="17.78" y="-2.54" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="6"/>
<pinref part="SVI6" gate="1" pin="6"/>
</segment>
<segment>
<pinref part="U30" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="-96.52" x2="414.02" y2="-96.52" width="0.1524" layer="91"/>
<label x="416.56" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA65" class="0">
<segment>
<wire x1="15.24" y1="-5.08" x2="27.94" y2="-5.08" width="0.1524" layer="91"/>
<label x="17.78" y="-5.08" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="5"/>
<pinref part="SVI6" gate="1" pin="5"/>
</segment>
<segment>
<pinref part="U30" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="-93.98" x2="414.02" y2="-93.98" width="0.1524" layer="91"/>
<label x="416.56" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA66" class="0">
<segment>
<wire x1="15.24" y1="-7.62" x2="27.94" y2="-7.62" width="0.1524" layer="91"/>
<label x="17.78" y="-7.62" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="4"/>
<pinref part="SVI6" gate="1" pin="4"/>
</segment>
<segment>
<pinref part="U31" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="-142.24" x2="414.02" y2="-142.24" width="0.1524" layer="91"/>
<label x="416.56" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA67" class="0">
<segment>
<wire x1="15.24" y1="-10.16" x2="27.94" y2="-10.16" width="0.1524" layer="91"/>
<label x="17.78" y="-10.16" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="3"/>
<pinref part="SVI6" gate="1" pin="3"/>
</segment>
<segment>
<pinref part="U31" gate="A" pin="IN_A"/>
<wire x1="429.26" y1="-139.7" x2="414.02" y2="-139.7" width="0.1524" layer="91"/>
<label x="416.56" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PA68" class="0">
<segment>
<wire x1="15.24" y1="-12.7" x2="27.94" y2="-12.7" width="0.1524" layer="91"/>
<label x="17.78" y="-12.7" size="1.778" layer="95"/>
<pinref part="SVO6" gate="1" pin="2"/>
<pinref part="SVI6" gate="1" pin="2"/>
</segment>
<segment>
<pinref part="U32" gate="A" pin="IN_B"/>
<wire x1="429.26" y1="-187.96" x2="414.02" y2="-187.96" width="0.1524" layer="91"/>
<label x="416.56" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="VIN" class="6">
<segment>
<wire x1="15.24" y1="40.64" x2="27.94" y2="40.64" width="0.1524" layer="91"/>
<label x="17.78" y="40.64" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="1"/>
<pinref part="SVI4" gate="1" pin="1"/>
</segment>
<segment>
<label x="99.06" y="119.38" size="1.778" layer="95"/>
<pinref part="VIN_ENABLE" gate="G$1" pin="3"/>
<wire x1="91.44" y1="119.38" x2="104.14" y2="119.38" width="0.1524" layer="91"/>
<wire x1="91.44" y1="121.92" x2="91.44" y2="119.38" width="0.1524" layer="91"/>
<pinref part="VIN_ENABLE" gate="G$1" pin="2"/>
<wire x1="88.9" y1="121.92" x2="88.9" y2="119.38" width="0.1524" layer="91"/>
<wire x1="88.9" y1="119.38" x2="91.44" y2="119.38" width="0.1524" layer="91"/>
<junction x="91.44" y="119.38"/>
</segment>
</net>
<net name="+3V3" class="3">
<segment>
<wire x1="15.24" y1="50.8" x2="27.94" y2="50.8" width="0.1524" layer="91"/>
<label x="17.78" y="50.8" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="5"/>
<pinref part="SVI4" gate="1" pin="5"/>
<wire x1="15.24" y1="50.8" x2="-2.54" y2="50.8" width="0.1524" layer="91"/>
<junction x="15.24" y="50.8"/>
<wire x1="-2.54" y1="50.8" x2="-2.54" y2="58.42" width="0.1524" layer="91"/>
<pinref part="3V3" gate="G$1" pin="+3V3"/>
</segment>
</net>
<net name="RESET" class="0">
<segment>
<wire x1="15.24" y1="53.34" x2="27.94" y2="53.34" width="0.1524" layer="91"/>
<label x="17.78" y="53.34" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="6"/>
<pinref part="SVI4" gate="1" pin="6"/>
</segment>
<segment>
<pinref part="RESET" gate="1" pin="P"/>
<wire x1="-10.16" y1="104.14" x2="-15.24" y2="104.14" width="0.1524" layer="91"/>
<pinref part="RESET" gate="1" pin="P1"/>
<wire x1="-15.24" y1="104.14" x2="-25.4" y2="104.14" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="101.6" x2="-15.24" y2="101.6" width="0.1524" layer="91"/>
<wire x1="-15.24" y1="101.6" x2="-15.24" y2="104.14" width="0.1524" layer="91"/>
<junction x="-15.24" y="104.14"/>
<label x="-25.4" y="104.14" size="1.778" layer="95"/>
</segment>
</net>
<net name="IOREF" class="0">
<segment>
<wire x1="15.24" y1="55.88" x2="27.94" y2="55.88" width="0.1524" layer="91"/>
<label x="17.78" y="55.88" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="7"/>
<pinref part="SVI4" gate="1" pin="7"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="D1" gate="1" pin="A"/>
<wire x1="25.4" y1="119.38" x2="16.51" y2="119.38" width="0.1524" layer="91"/>
<pinref part="PWR_IN" gate="J$1" pin="PWR"/>
<wire x1="16.51" y1="127" x2="16.51" y2="119.38" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="6">
<segment>
<pinref part="IC1" gate="A1" pin="VO"/>
<wire x1="66.04" y1="119.38" x2="73.66" y2="119.38" width="0.1524" layer="91"/>
<pinref part="VIN_ENABLE" gate="G$1" pin="1"/>
<wire x1="73.66" y1="119.38" x2="86.36" y2="119.38" width="0.1524" layer="91"/>
<wire x1="86.36" y1="119.38" x2="86.36" y2="121.92" width="0.1524" layer="91"/>
<wire x1="73.66" y1="116.84" x2="73.66" y2="119.38" width="0.1524" layer="91"/>
<junction x="73.66" y="119.38"/>
<pinref part="C66" gate="G$1" pin="1"/>
</segment>
</net>
<net name="PD7" class="4">
<segment>
<pinref part="U1" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="134.62" x2="210.82" y2="134.62" width="0.1524" layer="91"/>
<wire x1="210.82" y1="134.62" x2="210.82" y2="139.7" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="139.7" x2="215.9" y2="139.7" width="0.1524" layer="91"/>
<label x="200.66" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD9" class="4">
<segment>
<pinref part="U2" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="88.9" x2="210.82" y2="88.9" width="0.1524" layer="91"/>
<wire x1="210.82" y1="88.9" x2="210.82" y2="93.98" width="0.1524" layer="91"/>
<pinref part="U$6" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="93.98" x2="215.9" y2="93.98" width="0.1524" layer="91"/>
<label x="200.66" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD10" class="4">
<segment>
<pinref part="U3" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="43.18" x2="210.82" y2="43.18" width="0.1524" layer="91"/>
<wire x1="210.82" y1="43.18" x2="210.82" y2="48.26" width="0.1524" layer="91"/>
<pinref part="U$8" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="48.26" x2="215.9" y2="48.26" width="0.1524" layer="91"/>
<label x="200.66" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD11" class="4">
<segment>
<pinref part="U3" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="40.64" x2="210.82" y2="40.64" width="0.1524" layer="91"/>
<wire x1="210.82" y1="40.64" x2="210.82" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$7" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="33.02" x2="215.9" y2="33.02" width="0.1524" layer="91"/>
<label x="200.66" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD12" class="4">
<segment>
<pinref part="U4" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="-2.54" x2="210.82" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-2.54" x2="210.82" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$10" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="2.54" x2="215.9" y2="2.54" width="0.1524" layer="91"/>
<label x="200.66" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD13" class="4">
<segment>
<pinref part="U4" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="-5.08" x2="210.82" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-5.08" x2="210.82" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$9" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-12.7" x2="215.9" y2="-12.7" width="0.1524" layer="91"/>
<label x="200.66" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD14" class="4">
<segment>
<pinref part="U5" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="-48.26" x2="210.82" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-48.26" x2="210.82" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-43.18" x2="215.9" y2="-43.18" width="0.1524" layer="91"/>
<label x="200.66" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD15" class="4">
<segment>
<pinref part="U5" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="-50.8" x2="210.82" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-50.8" x2="210.82" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-58.42" x2="215.9" y2="-58.42" width="0.1524" layer="91"/>
<label x="200.66" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD16" class="4">
<segment>
<pinref part="U6" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="-93.98" x2="210.82" y2="-93.98" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-93.98" x2="210.82" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="U$12" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-88.9" x2="215.9" y2="-88.9" width="0.1524" layer="91"/>
<label x="200.66" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD17" class="4">
<segment>
<pinref part="U6" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="-96.52" x2="210.82" y2="-96.52" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-96.52" x2="210.82" y2="-104.14" width="0.1524" layer="91"/>
<pinref part="U$11" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-104.14" x2="215.9" y2="-104.14" width="0.1524" layer="91"/>
<label x="200.66" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD18" class="4">
<segment>
<pinref part="U7" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="-139.7" x2="210.82" y2="-139.7" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-139.7" x2="210.82" y2="-134.62" width="0.1524" layer="91"/>
<pinref part="U$14" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-134.62" x2="215.9" y2="-134.62" width="0.1524" layer="91"/>
<label x="200.66" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD19" class="4">
<segment>
<pinref part="U7" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="-142.24" x2="210.82" y2="-142.24" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-142.24" x2="210.82" y2="-149.86" width="0.1524" layer="91"/>
<pinref part="U$13" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-149.86" x2="215.9" y2="-149.86" width="0.1524" layer="91"/>
<label x="200.66" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD20" class="4">
<segment>
<pinref part="U8" gate="A" pin="!OUT_A"/>
<wire x1="195.58" y1="-185.42" x2="210.82" y2="-185.42" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-185.42" x2="210.82" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="U$16" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-180.34" x2="215.9" y2="-180.34" width="0.1524" layer="91"/>
<label x="200.66" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD21" class="4">
<segment>
<pinref part="U8" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="-187.96" x2="210.82" y2="-187.96" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-187.96" x2="210.82" y2="-195.58" width="0.1524" layer="91"/>
<pinref part="U$15" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="-195.58" x2="215.9" y2="-195.58" width="0.1524" layer="91"/>
<label x="200.66" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD24" class="4">
<segment>
<pinref part="U9" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="134.62" x2="302.26" y2="134.62" width="0.1524" layer="91"/>
<wire x1="302.26" y1="134.62" x2="302.26" y2="139.7" width="0.1524" layer="91"/>
<pinref part="U$18" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="139.7" x2="307.34" y2="139.7" width="0.1524" layer="91"/>
<label x="292.1" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD25" class="4">
<segment>
<pinref part="U10" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="86.36" x2="302.26" y2="86.36" width="0.1524" layer="91"/>
<wire x1="302.26" y1="86.36" x2="302.26" y2="78.74" width="0.1524" layer="91"/>
<pinref part="U$19" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="78.74" x2="307.34" y2="78.74" width="0.1524" layer="91"/>
<label x="292.1" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD28" class="4">
<segment>
<pinref part="U11" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="43.18" x2="302.26" y2="43.18" width="0.1524" layer="91"/>
<wire x1="302.26" y1="43.18" x2="302.26" y2="48.26" width="0.1524" layer="91"/>
<pinref part="U$22" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="48.26" x2="307.34" y2="48.26" width="0.1524" layer="91"/>
<label x="292.1" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD29" class="4">
<segment>
<pinref part="U12" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="-5.08" x2="302.26" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-5.08" x2="302.26" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$23" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-12.7" x2="307.34" y2="-12.7" width="0.1524" layer="91"/>
<label x="292.1" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD32" class="4">
<segment>
<pinref part="U13" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="-48.26" x2="302.26" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-48.26" x2="302.26" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="U$26" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-43.18" x2="307.34" y2="-43.18" width="0.1524" layer="91"/>
<label x="292.1" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD33" class="4">
<segment>
<pinref part="U14" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="-96.52" x2="302.26" y2="-96.52" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-96.52" x2="302.26" y2="-104.14" width="0.1524" layer="91"/>
<pinref part="U$27" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-104.14" x2="307.34" y2="-104.14" width="0.1524" layer="91"/>
<label x="292.1" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD34" class="4">
<segment>
<pinref part="U15" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="-142.24" x2="302.26" y2="-142.24" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-142.24" x2="302.26" y2="-149.86" width="0.1524" layer="91"/>
<pinref part="U$29" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-149.86" x2="307.34" y2="-149.86" width="0.1524" layer="91"/>
<label x="292.1" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD37" class="4">
<segment>
<pinref part="U16" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="-187.96" x2="302.26" y2="-187.96" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-187.96" x2="302.26" y2="-195.58" width="0.1524" layer="91"/>
<pinref part="U$31" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-195.58" x2="307.34" y2="-195.58" width="0.1524" layer="91"/>
<label x="292.1" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD38" class="4">
<segment>
<pinref part="U17" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="132.08" x2="391.16" y2="132.08" width="0.1524" layer="91"/>
<wire x1="391.16" y1="132.08" x2="391.16" y2="124.46" width="0.1524" layer="91"/>
<pinref part="U$33" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="124.46" x2="396.24" y2="124.46" width="0.1524" layer="91"/>
<label x="381" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD41" class="4">
<segment>
<pinref part="U18" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="86.36" x2="391.16" y2="86.36" width="0.1524" layer="91"/>
<wire x1="391.16" y1="86.36" x2="391.16" y2="78.74" width="0.1524" layer="91"/>
<pinref part="U$35" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="78.74" x2="396.24" y2="78.74" width="0.1524" layer="91"/>
<label x="381" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD44" class="4">
<segment>
<pinref part="U19" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="43.18" x2="391.16" y2="43.18" width="0.1524" layer="91"/>
<wire x1="391.16" y1="43.18" x2="391.16" y2="48.26" width="0.1524" layer="91"/>
<pinref part="U$38" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="48.26" x2="396.24" y2="48.26" width="0.1524" layer="91"/>
<label x="381" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD45" class="4">
<segment>
<pinref part="U20" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="-5.08" x2="391.16" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-5.08" x2="391.16" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$39" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-12.7" x2="396.24" y2="-12.7" width="0.1524" layer="91"/>
<label x="381" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD48" class="4">
<segment>
<pinref part="U21" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="-48.26" x2="391.16" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-48.26" x2="391.16" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="U$42" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-43.18" x2="396.24" y2="-43.18" width="0.1524" layer="91"/>
<label x="381" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD49" class="4">
<segment>
<pinref part="U22" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="-96.52" x2="391.16" y2="-96.52" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-96.52" x2="391.16" y2="-104.14" width="0.1524" layer="91"/>
<pinref part="U$43" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-104.14" x2="396.24" y2="-104.14" width="0.1524" layer="91"/>
<label x="381" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD52" class="4">
<segment>
<pinref part="U23" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="-139.7" x2="391.16" y2="-139.7" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-139.7" x2="391.16" y2="-134.62" width="0.1524" layer="91"/>
<pinref part="U$46" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-134.62" x2="396.24" y2="-134.62" width="0.1524" layer="91"/>
<label x="381" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD53" class="4">
<segment>
<pinref part="U24" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="-187.96" x2="391.16" y2="-187.96" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-187.96" x2="391.16" y2="-195.58" width="0.1524" layer="91"/>
<pinref part="U$47" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-195.58" x2="396.24" y2="-195.58" width="0.1524" layer="91"/>
<label x="381" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD54" class="4">
<segment>
<pinref part="U25" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="134.62" x2="480.06" y2="134.62" width="0.1524" layer="91"/>
<wire x1="480.06" y1="134.62" x2="480.06" y2="139.7" width="0.1524" layer="91"/>
<pinref part="U$50" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="139.7" x2="485.14" y2="139.7" width="0.1524" layer="91"/>
<label x="469.9" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD55" class="4">
<segment>
<pinref part="U25" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="132.08" x2="480.06" y2="132.08" width="0.1524" layer="91"/>
<wire x1="480.06" y1="132.08" x2="480.06" y2="124.46" width="0.1524" layer="91"/>
<pinref part="U$49" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="124.46" x2="485.14" y2="124.46" width="0.1524" layer="91"/>
<label x="469.9" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD56" class="4">
<segment>
<pinref part="U26" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="88.9" x2="480.06" y2="88.9" width="0.1524" layer="91"/>
<wire x1="480.06" y1="88.9" x2="480.06" y2="93.98" width="0.1524" layer="91"/>
<pinref part="U$52" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="93.98" x2="485.14" y2="93.98" width="0.1524" layer="91"/>
<label x="469.9" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD57" class="4">
<segment>
<pinref part="U26" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="86.36" x2="480.06" y2="86.36" width="0.1524" layer="91"/>
<wire x1="480.06" y1="86.36" x2="480.06" y2="78.74" width="0.1524" layer="91"/>
<pinref part="U$51" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="78.74" x2="485.14" y2="78.74" width="0.1524" layer="91"/>
<label x="469.9" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD58" class="4">
<segment>
<pinref part="U27" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="43.18" x2="480.06" y2="43.18" width="0.1524" layer="91"/>
<wire x1="480.06" y1="43.18" x2="480.06" y2="48.26" width="0.1524" layer="91"/>
<pinref part="U$54" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="48.26" x2="485.14" y2="48.26" width="0.1524" layer="91"/>
<label x="469.9" y="43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD59" class="4">
<segment>
<pinref part="U27" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="40.64" x2="480.06" y2="40.64" width="0.1524" layer="91"/>
<wire x1="480.06" y1="40.64" x2="480.06" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$53" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="33.02" x2="485.14" y2="33.02" width="0.1524" layer="91"/>
<label x="469.9" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD60" class="4">
<segment>
<pinref part="U28" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="-2.54" x2="480.06" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-2.54" x2="480.06" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$56" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="2.54" x2="485.14" y2="2.54" width="0.1524" layer="91"/>
<label x="469.9" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD61" class="4">
<segment>
<pinref part="U28" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="-5.08" x2="480.06" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-5.08" x2="480.06" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="U$55" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-12.7" x2="485.14" y2="-12.7" width="0.1524" layer="91"/>
<label x="469.9" y="-5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD62" class="4">
<segment>
<pinref part="U29" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="-48.26" x2="480.06" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-48.26" x2="480.06" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="U$58" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-43.18" x2="485.14" y2="-43.18" width="0.1524" layer="91"/>
<label x="469.9" y="-48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD63" class="4">
<segment>
<pinref part="U29" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="-50.8" x2="480.06" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-50.8" x2="480.06" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="U$57" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-58.42" x2="485.14" y2="-58.42" width="0.1524" layer="91"/>
<label x="469.9" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD65" class="4">
<segment>
<pinref part="U30" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="-93.98" x2="480.06" y2="-93.98" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-93.98" x2="480.06" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="U$60" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-88.9" x2="485.14" y2="-88.9" width="0.1524" layer="91"/>
<label x="469.9" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD67" class="4">
<segment>
<pinref part="U31" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="-139.7" x2="480.06" y2="-139.7" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-139.7" x2="480.06" y2="-134.62" width="0.1524" layer="91"/>
<pinref part="U$62" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-134.62" x2="485.14" y2="-134.62" width="0.1524" layer="91"/>
<label x="469.9" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD69" class="4">
<segment>
<pinref part="U32" gate="A" pin="!OUT_A"/>
<wire x1="464.82" y1="-185.42" x2="480.06" y2="-185.42" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-185.42" x2="480.06" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="U$64" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-180.34" x2="485.14" y2="-180.34" width="0.1524" layer="91"/>
<label x="469.9" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD23" class="4">
<segment>
<pinref part="U10" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="88.9" x2="302.26" y2="88.9" width="0.1524" layer="91"/>
<wire x1="302.26" y1="88.9" x2="302.26" y2="93.98" width="0.1524" layer="91"/>
<pinref part="U$20" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="93.98" x2="307.34" y2="93.98" width="0.1524" layer="91"/>
<label x="292.1" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD27" class="4">
<segment>
<pinref part="U12" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="-2.54" x2="302.26" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-2.54" x2="302.26" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$24" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="2.54" x2="307.34" y2="2.54" width="0.1524" layer="91"/>
<label x="292.1" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD31" class="4">
<segment>
<pinref part="U14" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="-93.98" x2="302.26" y2="-93.98" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-93.98" x2="302.26" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="U$28" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-88.9" x2="307.34" y2="-88.9" width="0.1524" layer="91"/>
<label x="292.1" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD35" class="4">
<segment>
<pinref part="U16" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="-185.42" x2="302.26" y2="-185.42" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-185.42" x2="302.26" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="U$32" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-180.34" x2="307.34" y2="-180.34" width="0.1524" layer="91"/>
<label x="292.1" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD39" class="4">
<segment>
<pinref part="U18" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="88.9" x2="391.16" y2="88.9" width="0.1524" layer="91"/>
<wire x1="391.16" y1="88.9" x2="391.16" y2="93.98" width="0.1524" layer="91"/>
<pinref part="U$36" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="93.98" x2="396.24" y2="93.98" width="0.1524" layer="91"/>
<label x="381" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD43" class="4">
<segment>
<pinref part="U20" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="-2.54" x2="391.16" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-2.54" x2="391.16" y2="2.54" width="0.1524" layer="91"/>
<pinref part="U$40" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="2.54" x2="396.24" y2="2.54" width="0.1524" layer="91"/>
<label x="381" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD47" class="4">
<segment>
<pinref part="U22" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="-93.98" x2="391.16" y2="-93.98" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-93.98" x2="391.16" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="U$44" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-88.9" x2="396.24" y2="-88.9" width="0.1524" layer="91"/>
<label x="381" y="-93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD51" class="4">
<segment>
<pinref part="U24" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="-185.42" x2="391.16" y2="-185.42" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-185.42" x2="391.16" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="U$48" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-180.34" x2="396.24" y2="-180.34" width="0.1524" layer="91"/>
<label x="381" y="-185.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="+5V" class="2">
<segment>
<wire x1="15.24" y1="48.26" x2="27.94" y2="48.26" width="0.1524" layer="91"/>
<label x="17.78" y="48.26" size="1.778" layer="95"/>
<pinref part="SVO4" gate="1" pin="4"/>
<pinref part="SVI4" gate="1" pin="4"/>
<wire x1="15.24" y1="48.26" x2="-7.62" y2="48.26" width="0.1524" layer="91"/>
<junction x="15.24" y="48.26"/>
<wire x1="-7.62" y1="48.26" x2="-7.62" y2="58.42" width="0.1524" layer="91"/>
<pinref part="+5V" gate="1" pin="+5V"/>
</segment>
<segment>
<pinref part="SVI7" gate="G$1" pin="1"/>
<label x="83.82" y="-15.24" size="1.778" layer="95" rot="R90"/>
<wire x1="83.82" y1="-20.32" x2="91.44" y2="-20.32" width="0.1524" layer="91"/>
<wire x1="91.44" y1="-20.32" x2="91.44" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="83.82" y1="-48.26" x2="83.82" y2="-35.56" width="0.1524" layer="91"/>
<label x="83.82" y="-45.72" size="1.778" layer="95" rot="R90"/>
<pinref part="SVI7" gate="G$1" pin="2"/>
<wire x1="91.44" y1="-35.56" x2="83.82" y2="-35.56" width="0.1524" layer="91"/>
<junction x="83.82" y="-35.56"/>
<junction x="83.82" y="-20.32"/>
<pinref part="V1" gate="1" pin="+5V"/>
<wire x1="91.44" y1="-35.56" x2="96.52" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="96.52" y1="-35.56" x2="96.52" y2="-30.48" width="0.1524" layer="91"/>
<junction x="91.44" y="-35.56"/>
<wire x1="83.82" y1="-20.32" x2="83.82" y2="7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+18V" class="1">
<segment>
<pinref part="IC1" gate="A1" pin="VI"/>
<pinref part="D1" gate="1" pin="C"/>
<wire x1="45.72" y1="119.38" x2="40.64" y2="119.38" width="0.1524" layer="91"/>
<wire x1="40.64" y1="119.38" x2="30.48" y2="119.38" width="0.1524" layer="91"/>
<wire x1="40.64" y1="116.84" x2="40.64" y2="119.38" width="0.1524" layer="91"/>
<junction x="40.64" y="119.38"/>
<wire x1="40.64" y1="119.38" x2="40.64" y2="127" width="0.1524" layer="91"/>
<pinref part="V2" gate="1" pin="+18V"/>
<label x="40.64" y="124.46" size="1.778" layer="95"/>
<pinref part="C65" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="V4" gate="1" pin="+18V"/>
<wire x1="200.66" y1="139.7" x2="200.66" y2="144.78" width="0.1524" layer="91"/>
<label x="203.2" y="139.7" size="1.778" layer="95"/>
<pinref part="U1" gate="A" pin="VDD"/>
<wire x1="195.58" y1="139.7" x2="198.12" y2="139.7" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="1"/>
<wire x1="198.12" y1="139.7" x2="200.66" y2="139.7" width="0.1524" layer="91"/>
<wire x1="198.12" y1="129.54" x2="198.12" y2="139.7" width="0.1524" layer="91"/>
<junction x="198.12" y="139.7"/>
</segment>
<segment>
<pinref part="C33" gate="G$1" pin="1"/>
<pinref part="V36" gate="1" pin="+18V"/>
<wire x1="228.6" y1="129.54" x2="228.6" y2="127" width="0.1524" layer="91"/>
<junction x="228.6" y="127"/>
</segment>
<segment>
<wire x1="200.66" y1="93.98" x2="200.66" y2="99.06" width="0.1524" layer="91"/>
<pinref part="V6" gate="1" pin="+18V"/>
<label x="203.2" y="93.98" size="1.778" layer="95"/>
<pinref part="U2" gate="A" pin="VDD"/>
<wire x1="195.58" y1="93.98" x2="198.12" y2="93.98" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="1"/>
<wire x1="198.12" y1="93.98" x2="200.66" y2="93.98" width="0.1524" layer="91"/>
<wire x1="198.12" y1="83.82" x2="198.12" y2="93.98" width="0.1524" layer="91"/>
<junction x="198.12" y="93.98"/>
</segment>
<segment>
<pinref part="C34" gate="G$1" pin="1"/>
<pinref part="V37" gate="1" pin="+18V"/>
<wire x1="228.6" y1="83.82" x2="228.6" y2="81.28" width="0.1524" layer="91"/>
<junction x="228.6" y="81.28"/>
</segment>
<segment>
<pinref part="V12" gate="1" pin="+18V"/>
<wire x1="292.1" y1="139.7" x2="292.1" y2="144.78" width="0.1524" layer="91"/>
<label x="294.64" y="139.7" size="1.778" layer="95"/>
<pinref part="U9" gate="A" pin="VDD"/>
<wire x1="287.02" y1="139.7" x2="289.56" y2="139.7" width="0.1524" layer="91"/>
<pinref part="C9" gate="G$1" pin="1"/>
<wire x1="289.56" y1="139.7" x2="292.1" y2="139.7" width="0.1524" layer="91"/>
<wire x1="289.56" y1="129.54" x2="289.56" y2="139.7" width="0.1524" layer="91"/>
<junction x="289.56" y="139.7"/>
</segment>
<segment>
<wire x1="292.1" y1="93.98" x2="292.1" y2="99.06" width="0.1524" layer="91"/>
<pinref part="V13" gate="1" pin="+18V"/>
<label x="294.64" y="93.98" size="1.778" layer="95"/>
<pinref part="U10" gate="A" pin="VDD"/>
<wire x1="287.02" y1="93.98" x2="289.56" y2="93.98" width="0.1524" layer="91"/>
<pinref part="C10" gate="G$1" pin="1"/>
<wire x1="289.56" y1="93.98" x2="292.1" y2="93.98" width="0.1524" layer="91"/>
<wire x1="289.56" y1="83.82" x2="289.56" y2="93.98" width="0.1524" layer="91"/>
<junction x="289.56" y="93.98"/>
</segment>
<segment>
<pinref part="C41" gate="G$1" pin="1"/>
<pinref part="V44" gate="1" pin="+18V"/>
<wire x1="320.04" y1="129.54" x2="320.04" y2="127" width="0.1524" layer="91"/>
<junction x="320.04" y="127"/>
</segment>
<segment>
<pinref part="C42" gate="G$1" pin="1"/>
<pinref part="V45" gate="1" pin="+18V"/>
<wire x1="320.04" y1="83.82" x2="320.04" y2="81.28" width="0.1524" layer="91"/>
<junction x="320.04" y="81.28"/>
</segment>
<segment>
<pinref part="C43" gate="G$1" pin="1"/>
<pinref part="V46" gate="1" pin="+18V"/>
<wire x1="320.04" y1="38.1" x2="320.04" y2="35.56" width="0.1524" layer="91"/>
<junction x="320.04" y="35.56"/>
</segment>
<segment>
<pinref part="C44" gate="G$1" pin="1"/>
<pinref part="V47" gate="1" pin="+18V"/>
<wire x1="320.04" y1="-7.62" x2="320.04" y2="-10.16" width="0.1524" layer="91"/>
<junction x="320.04" y="-10.16"/>
</segment>
<segment>
<pinref part="C45" gate="G$1" pin="1"/>
<pinref part="V48" gate="1" pin="+18V"/>
<wire x1="320.04" y1="-53.34" x2="320.04" y2="-55.88" width="0.1524" layer="91"/>
<junction x="320.04" y="-55.88"/>
</segment>
<segment>
<pinref part="C37" gate="G$1" pin="1"/>
<pinref part="V40" gate="1" pin="+18V"/>
<wire x1="228.6" y1="-53.34" x2="228.6" y2="-55.88" width="0.1524" layer="91"/>
<junction x="228.6" y="-55.88"/>
</segment>
<segment>
<pinref part="C36" gate="G$1" pin="1"/>
<pinref part="V39" gate="1" pin="+18V"/>
<wire x1="228.6" y1="-7.62" x2="228.6" y2="-10.16" width="0.1524" layer="91"/>
<junction x="228.6" y="-10.16"/>
</segment>
<segment>
<pinref part="C35" gate="G$1" pin="1"/>
<pinref part="V38" gate="1" pin="+18V"/>
<wire x1="228.6" y1="38.1" x2="228.6" y2="35.56" width="0.1524" layer="91"/>
<junction x="228.6" y="35.56"/>
</segment>
<segment>
<pinref part="U3" gate="A" pin="VDD"/>
<wire x1="195.58" y1="48.26" x2="198.12" y2="48.26" width="0.1524" layer="91"/>
<wire x1="198.12" y1="48.26" x2="200.66" y2="48.26" width="0.1524" layer="91"/>
<wire x1="200.66" y1="48.26" x2="200.66" y2="53.34" width="0.1524" layer="91"/>
<pinref part="V7" gate="1" pin="+18V"/>
<label x="203.2" y="48.26" size="1.778" layer="95"/>
<pinref part="C3" gate="G$1" pin="1"/>
<wire x1="198.12" y1="38.1" x2="198.12" y2="48.26" width="0.1524" layer="91"/>
<junction x="198.12" y="48.26"/>
</segment>
<segment>
<pinref part="U11" gate="A" pin="VDD"/>
<wire x1="287.02" y1="48.26" x2="289.56" y2="48.26" width="0.1524" layer="91"/>
<wire x1="289.56" y1="48.26" x2="292.1" y2="48.26" width="0.1524" layer="91"/>
<wire x1="292.1" y1="48.26" x2="292.1" y2="53.34" width="0.1524" layer="91"/>
<pinref part="V14" gate="1" pin="+18V"/>
<label x="294.64" y="48.26" size="1.778" layer="95"/>
<pinref part="C11" gate="G$1" pin="1"/>
<wire x1="289.56" y1="38.1" x2="289.56" y2="48.26" width="0.1524" layer="91"/>
<junction x="289.56" y="48.26"/>
</segment>
<segment>
<wire x1="292.1" y1="2.54" x2="292.1" y2="7.62" width="0.1524" layer="91"/>
<pinref part="V15" gate="1" pin="+18V"/>
<label x="294.64" y="2.54" size="1.778" layer="95"/>
<pinref part="U12" gate="A" pin="VDD"/>
<wire x1="287.02" y1="2.54" x2="289.56" y2="2.54" width="0.1524" layer="91"/>
<pinref part="C12" gate="G$1" pin="1"/>
<wire x1="289.56" y1="2.54" x2="292.1" y2="2.54" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-7.62" x2="289.56" y2="2.54" width="0.1524" layer="91"/>
<junction x="289.56" y="2.54"/>
</segment>
<segment>
<pinref part="V16" gate="1" pin="+18V"/>
<wire x1="292.1" y1="-43.18" x2="292.1" y2="-38.1" width="0.1524" layer="91"/>
<label x="294.64" y="-43.18" size="1.778" layer="95"/>
<pinref part="U13" gate="A" pin="VDD"/>
<wire x1="287.02" y1="-43.18" x2="289.56" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="C13" gate="G$1" pin="1"/>
<wire x1="289.56" y1="-43.18" x2="292.1" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-53.34" x2="289.56" y2="-43.18" width="0.1524" layer="91"/>
<junction x="289.56" y="-43.18"/>
</segment>
<segment>
<pinref part="V3" gate="1" pin="+18V"/>
<wire x1="200.66" y1="-43.18" x2="200.66" y2="-38.1" width="0.1524" layer="91"/>
<label x="203.2" y="-43.18" size="1.778" layer="95"/>
<pinref part="U5" gate="A" pin="VDD"/>
<wire x1="195.58" y1="-43.18" x2="198.12" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="C5" gate="G$1" pin="1"/>
<wire x1="198.12" y1="-43.18" x2="200.66" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-53.34" x2="198.12" y2="-43.18" width="0.1524" layer="91"/>
<junction x="198.12" y="-43.18"/>
</segment>
<segment>
<wire x1="200.66" y1="2.54" x2="200.66" y2="7.62" width="0.1524" layer="91"/>
<pinref part="V8" gate="1" pin="+18V"/>
<label x="203.2" y="2.54" size="1.778" layer="95"/>
<pinref part="U4" gate="A" pin="VDD"/>
<wire x1="195.58" y1="2.54" x2="198.12" y2="2.54" width="0.1524" layer="91"/>
<pinref part="C4" gate="G$1" pin="1"/>
<wire x1="198.12" y1="2.54" x2="200.66" y2="2.54" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-7.62" x2="198.12" y2="2.54" width="0.1524" layer="91"/>
<junction x="198.12" y="2.54"/>
</segment>
<segment>
<wire x1="200.66" y1="-88.9" x2="200.66" y2="-83.82" width="0.1524" layer="91"/>
<pinref part="V9" gate="1" pin="+18V"/>
<label x="203.2" y="-88.9" size="1.778" layer="95"/>
<pinref part="U6" gate="A" pin="VDD"/>
<wire x1="195.58" y1="-88.9" x2="198.12" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="C6" gate="G$1" pin="1"/>
<wire x1="198.12" y1="-88.9" x2="200.66" y2="-88.9" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-99.06" x2="198.12" y2="-88.9" width="0.1524" layer="91"/>
<junction x="198.12" y="-88.9"/>
</segment>
<segment>
<pinref part="U7" gate="A" pin="VDD"/>
<wire x1="195.58" y1="-134.62" x2="198.12" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-134.62" x2="200.66" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="200.66" y1="-134.62" x2="200.66" y2="-129.54" width="0.1524" layer="91"/>
<pinref part="V10" gate="1" pin="+18V"/>
<label x="203.2" y="-134.62" size="1.778" layer="95"/>
<pinref part="C7" gate="G$1" pin="1"/>
<wire x1="198.12" y1="-144.78" x2="198.12" y2="-134.62" width="0.1524" layer="91"/>
<junction x="198.12" y="-134.62"/>
</segment>
<segment>
<wire x1="200.66" y1="-180.34" x2="200.66" y2="-175.26" width="0.1524" layer="91"/>
<pinref part="V11" gate="1" pin="+18V"/>
<label x="203.2" y="-180.34" size="1.778" layer="95"/>
<pinref part="U8" gate="A" pin="VDD"/>
<wire x1="195.58" y1="-180.34" x2="198.12" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="C8" gate="G$1" pin="1"/>
<wire x1="198.12" y1="-180.34" x2="200.66" y2="-180.34" width="0.1524" layer="91"/>
<wire x1="198.12" y1="-190.5" x2="198.12" y2="-180.34" width="0.1524" layer="91"/>
<junction x="198.12" y="-180.34"/>
</segment>
<segment>
<wire x1="292.1" y1="-180.34" x2="292.1" y2="-175.26" width="0.1524" layer="91"/>
<pinref part="V19" gate="1" pin="+18V"/>
<label x="294.64" y="-180.34" size="1.778" layer="95"/>
<pinref part="U16" gate="A" pin="VDD"/>
<wire x1="287.02" y1="-180.34" x2="289.56" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="C16" gate="G$1" pin="1"/>
<wire x1="289.56" y1="-180.34" x2="292.1" y2="-180.34" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-190.5" x2="289.56" y2="-180.34" width="0.1524" layer="91"/>
<junction x="289.56" y="-180.34"/>
</segment>
<segment>
<pinref part="U15" gate="A" pin="VDD"/>
<wire x1="287.02" y1="-134.62" x2="289.56" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-134.62" x2="292.1" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="292.1" y1="-134.62" x2="292.1" y2="-129.54" width="0.1524" layer="91"/>
<pinref part="V18" gate="1" pin="+18V"/>
<label x="294.64" y="-134.62" size="1.778" layer="95"/>
<pinref part="C15" gate="G$1" pin="1"/>
<wire x1="289.56" y1="-144.78" x2="289.56" y2="-134.62" width="0.1524" layer="91"/>
<junction x="289.56" y="-134.62"/>
</segment>
<segment>
<wire x1="292.1" y1="-88.9" x2="292.1" y2="-83.82" width="0.1524" layer="91"/>
<pinref part="V17" gate="1" pin="+18V"/>
<label x="294.64" y="-88.9" size="1.778" layer="95"/>
<pinref part="U14" gate="A" pin="VDD"/>
<wire x1="287.02" y1="-88.9" x2="289.56" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="C14" gate="G$1" pin="1"/>
<wire x1="289.56" y1="-88.9" x2="292.1" y2="-88.9" width="0.1524" layer="91"/>
<wire x1="289.56" y1="-99.06" x2="289.56" y2="-88.9" width="0.1524" layer="91"/>
<junction x="289.56" y="-88.9"/>
</segment>
<segment>
<pinref part="C46" gate="G$1" pin="1"/>
<pinref part="V49" gate="1" pin="+18V"/>
<wire x1="320.04" y1="-99.06" x2="320.04" y2="-101.6" width="0.1524" layer="91"/>
<junction x="320.04" y="-101.6"/>
</segment>
<segment>
<pinref part="C38" gate="G$1" pin="1"/>
<pinref part="V41" gate="1" pin="+18V"/>
<wire x1="228.6" y1="-99.06" x2="228.6" y2="-101.6" width="0.1524" layer="91"/>
<junction x="228.6" y="-101.6"/>
</segment>
<segment>
<pinref part="C39" gate="G$1" pin="1"/>
<pinref part="V42" gate="1" pin="+18V"/>
<wire x1="228.6" y1="-144.78" x2="228.6" y2="-147.32" width="0.1524" layer="91"/>
<junction x="228.6" y="-147.32"/>
</segment>
<segment>
<pinref part="C47" gate="G$1" pin="1"/>
<pinref part="V50" gate="1" pin="+18V"/>
<wire x1="320.04" y1="-144.78" x2="320.04" y2="-147.32" width="0.1524" layer="91"/>
<junction x="320.04" y="-147.32"/>
</segment>
<segment>
<pinref part="C40" gate="G$1" pin="1"/>
<pinref part="V43" gate="1" pin="+18V"/>
<wire x1="228.6" y1="-190.5" x2="228.6" y2="-193.04" width="0.1524" layer="91"/>
<junction x="228.6" y="-193.04"/>
</segment>
<segment>
<pinref part="C48" gate="G$1" pin="1"/>
<pinref part="V51" gate="1" pin="+18V"/>
<wire x1="320.04" y1="-190.5" x2="320.04" y2="-193.04" width="0.1524" layer="91"/>
<junction x="320.04" y="-193.04"/>
</segment>
<segment>
<wire x1="381" y1="-180.34" x2="381" y2="-175.26" width="0.1524" layer="91"/>
<pinref part="V27" gate="1" pin="+18V"/>
<label x="383.54" y="-180.34" size="1.778" layer="95"/>
<pinref part="U24" gate="A" pin="VDD"/>
<wire x1="375.92" y1="-180.34" x2="378.46" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="C24" gate="G$1" pin="1"/>
<wire x1="378.46" y1="-180.34" x2="381" y2="-180.34" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-190.5" x2="378.46" y2="-180.34" width="0.1524" layer="91"/>
<junction x="378.46" y="-180.34"/>
</segment>
<segment>
<pinref part="U23" gate="A" pin="VDD"/>
<wire x1="375.92" y1="-134.62" x2="378.46" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-134.62" x2="381" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="381" y1="-134.62" x2="381" y2="-129.54" width="0.1524" layer="91"/>
<pinref part="V26" gate="1" pin="+18V"/>
<label x="383.54" y="-134.62" size="1.778" layer="95"/>
<pinref part="C23" gate="G$1" pin="1"/>
<wire x1="378.46" y1="-144.78" x2="378.46" y2="-134.62" width="0.1524" layer="91"/>
<junction x="378.46" y="-134.62"/>
</segment>
<segment>
<wire x1="381" y1="-88.9" x2="381" y2="-83.82" width="0.1524" layer="91"/>
<pinref part="V25" gate="1" pin="+18V"/>
<label x="383.54" y="-88.9" size="1.778" layer="95"/>
<pinref part="U22" gate="A" pin="VDD"/>
<wire x1="375.92" y1="-88.9" x2="378.46" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="C22" gate="G$1" pin="1"/>
<wire x1="378.46" y1="-88.9" x2="381" y2="-88.9" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-99.06" x2="378.46" y2="-88.9" width="0.1524" layer="91"/>
<junction x="378.46" y="-88.9"/>
</segment>
<segment>
<wire x1="469.9" y1="-88.9" x2="469.9" y2="-83.82" width="0.1524" layer="91"/>
<pinref part="V33" gate="1" pin="+18V"/>
<label x="472.44" y="-88.9" size="1.778" layer="95"/>
<pinref part="U30" gate="A" pin="VDD"/>
<wire x1="464.82" y1="-88.9" x2="467.36" y2="-88.9" width="0.1524" layer="91"/>
<pinref part="C30" gate="G$1" pin="1"/>
<wire x1="467.36" y1="-88.9" x2="469.9" y2="-88.9" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-99.06" x2="467.36" y2="-88.9" width="0.1524" layer="91"/>
<junction x="467.36" y="-88.9"/>
</segment>
<segment>
<pinref part="U31" gate="A" pin="VDD"/>
<wire x1="464.82" y1="-134.62" x2="467.36" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-134.62" x2="469.9" y2="-134.62" width="0.1524" layer="91"/>
<wire x1="469.9" y1="-134.62" x2="469.9" y2="-129.54" width="0.1524" layer="91"/>
<pinref part="V34" gate="1" pin="+18V"/>
<label x="472.44" y="-134.62" size="1.778" layer="95"/>
<pinref part="C31" gate="G$1" pin="1"/>
<wire x1="467.36" y1="-144.78" x2="467.36" y2="-134.62" width="0.1524" layer="91"/>
<junction x="467.36" y="-134.62"/>
</segment>
<segment>
<wire x1="469.9" y1="-180.34" x2="469.9" y2="-175.26" width="0.1524" layer="91"/>
<pinref part="V35" gate="1" pin="+18V"/>
<label x="472.44" y="-180.34" size="1.778" layer="95"/>
<pinref part="U32" gate="A" pin="VDD"/>
<wire x1="464.82" y1="-180.34" x2="467.36" y2="-180.34" width="0.1524" layer="91"/>
<pinref part="C32" gate="G$1" pin="1"/>
<wire x1="467.36" y1="-180.34" x2="469.9" y2="-180.34" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-190.5" x2="467.36" y2="-180.34" width="0.1524" layer="91"/>
<junction x="467.36" y="-180.34"/>
</segment>
<segment>
<pinref part="C64" gate="G$1" pin="1"/>
<pinref part="V67" gate="1" pin="+18V"/>
<wire x1="497.84" y1="-190.5" x2="497.84" y2="-193.04" width="0.1524" layer="91"/>
<junction x="497.84" y="-193.04"/>
</segment>
<segment>
<pinref part="C56" gate="G$1" pin="1"/>
<pinref part="V59" gate="1" pin="+18V"/>
<wire x1="408.94" y1="-190.5" x2="408.94" y2="-193.04" width="0.1524" layer="91"/>
<junction x="408.94" y="-193.04"/>
</segment>
<segment>
<pinref part="C55" gate="G$1" pin="1"/>
<pinref part="V58" gate="1" pin="+18V"/>
<wire x1="408.94" y1="-144.78" x2="408.94" y2="-147.32" width="0.1524" layer="91"/>
<junction x="408.94" y="-147.32"/>
</segment>
<segment>
<pinref part="C63" gate="G$1" pin="1"/>
<pinref part="V66" gate="1" pin="+18V"/>
<wire x1="497.84" y1="-144.78" x2="497.84" y2="-147.32" width="0.1524" layer="91"/>
<junction x="497.84" y="-147.32"/>
</segment>
<segment>
<pinref part="C62" gate="G$1" pin="1"/>
<pinref part="V65" gate="1" pin="+18V"/>
<wire x1="497.84" y1="-99.06" x2="497.84" y2="-101.6" width="0.1524" layer="91"/>
<junction x="497.84" y="-101.6"/>
</segment>
<segment>
<pinref part="C54" gate="G$1" pin="1"/>
<pinref part="V57" gate="1" pin="+18V"/>
<wire x1="408.94" y1="-99.06" x2="408.94" y2="-101.6" width="0.1524" layer="91"/>
<junction x="408.94" y="-101.6"/>
</segment>
<segment>
<pinref part="V24" gate="1" pin="+18V"/>
<wire x1="381" y1="-43.18" x2="381" y2="-38.1" width="0.1524" layer="91"/>
<label x="383.54" y="-43.18" size="1.778" layer="95"/>
<pinref part="U21" gate="A" pin="VDD"/>
<wire x1="375.92" y1="-43.18" x2="378.46" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="C21" gate="G$1" pin="1"/>
<wire x1="378.46" y1="-43.18" x2="381" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-53.34" x2="378.46" y2="-43.18" width="0.1524" layer="91"/>
<junction x="378.46" y="-43.18"/>
</segment>
<segment>
<wire x1="381" y1="2.54" x2="381" y2="7.62" width="0.1524" layer="91"/>
<pinref part="V23" gate="1" pin="+18V"/>
<label x="383.54" y="2.54" size="1.778" layer="95"/>
<pinref part="U20" gate="A" pin="VDD"/>
<wire x1="375.92" y1="2.54" x2="378.46" y2="2.54" width="0.1524" layer="91"/>
<pinref part="C20" gate="G$1" pin="1"/>
<wire x1="378.46" y1="2.54" x2="381" y2="2.54" width="0.1524" layer="91"/>
<wire x1="378.46" y1="-7.62" x2="378.46" y2="2.54" width="0.1524" layer="91"/>
<junction x="378.46" y="2.54"/>
</segment>
<segment>
<pinref part="U19" gate="A" pin="VDD"/>
<wire x1="375.92" y1="48.26" x2="378.46" y2="48.26" width="0.1524" layer="91"/>
<wire x1="378.46" y1="48.26" x2="381" y2="48.26" width="0.1524" layer="91"/>
<wire x1="381" y1="48.26" x2="381" y2="53.34" width="0.1524" layer="91"/>
<pinref part="V22" gate="1" pin="+18V"/>
<label x="383.54" y="48.26" size="1.778" layer="95"/>
<pinref part="C19" gate="G$1" pin="1"/>
<wire x1="378.46" y1="38.1" x2="378.46" y2="48.26" width="0.1524" layer="91"/>
<junction x="378.46" y="48.26"/>
</segment>
<segment>
<pinref part="U27" gate="A" pin="VDD"/>
<wire x1="464.82" y1="48.26" x2="467.36" y2="48.26" width="0.1524" layer="91"/>
<wire x1="467.36" y1="48.26" x2="469.9" y2="48.26" width="0.1524" layer="91"/>
<wire x1="469.9" y1="48.26" x2="469.9" y2="53.34" width="0.1524" layer="91"/>
<pinref part="V30" gate="1" pin="+18V"/>
<label x="472.44" y="48.26" size="1.778" layer="95"/>
<pinref part="C27" gate="G$1" pin="1"/>
<wire x1="467.36" y1="38.1" x2="467.36" y2="48.26" width="0.1524" layer="91"/>
<junction x="467.36" y="48.26"/>
</segment>
<segment>
<wire x1="469.9" y1="2.54" x2="469.9" y2="7.62" width="0.1524" layer="91"/>
<pinref part="V31" gate="1" pin="+18V"/>
<label x="472.44" y="2.54" size="1.778" layer="95"/>
<pinref part="U28" gate="A" pin="VDD"/>
<wire x1="464.82" y1="2.54" x2="467.36" y2="2.54" width="0.1524" layer="91"/>
<pinref part="C28" gate="G$1" pin="1"/>
<wire x1="467.36" y1="2.54" x2="469.9" y2="2.54" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-7.62" x2="467.36" y2="2.54" width="0.1524" layer="91"/>
<junction x="467.36" y="2.54"/>
</segment>
<segment>
<pinref part="V32" gate="1" pin="+18V"/>
<wire x1="469.9" y1="-43.18" x2="469.9" y2="-38.1" width="0.1524" layer="91"/>
<label x="472.44" y="-43.18" size="1.778" layer="95"/>
<pinref part="U29" gate="A" pin="VDD"/>
<wire x1="464.82" y1="-43.18" x2="467.36" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="C29" gate="G$1" pin="1"/>
<wire x1="467.36" y1="-43.18" x2="469.9" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="467.36" y1="-53.34" x2="467.36" y2="-43.18" width="0.1524" layer="91"/>
<junction x="467.36" y="-43.18"/>
</segment>
<segment>
<pinref part="C61" gate="G$1" pin="1"/>
<pinref part="V64" gate="1" pin="+18V"/>
<wire x1="497.84" y1="-53.34" x2="497.84" y2="-55.88" width="0.1524" layer="91"/>
<junction x="497.84" y="-55.88"/>
</segment>
<segment>
<pinref part="C53" gate="G$1" pin="1"/>
<pinref part="V56" gate="1" pin="+18V"/>
<wire x1="408.94" y1="-53.34" x2="408.94" y2="-55.88" width="0.1524" layer="91"/>
<junction x="408.94" y="-55.88"/>
</segment>
<segment>
<pinref part="C52" gate="G$1" pin="1"/>
<pinref part="V55" gate="1" pin="+18V"/>
<wire x1="408.94" y1="-7.62" x2="408.94" y2="-10.16" width="0.1524" layer="91"/>
<junction x="408.94" y="-10.16"/>
</segment>
<segment>
<pinref part="C60" gate="G$1" pin="1"/>
<pinref part="V63" gate="1" pin="+18V"/>
<wire x1="497.84" y1="-7.62" x2="497.84" y2="-10.16" width="0.1524" layer="91"/>
<junction x="497.84" y="-10.16"/>
</segment>
<segment>
<pinref part="C51" gate="G$1" pin="1"/>
<pinref part="V54" gate="1" pin="+18V"/>
<wire x1="408.94" y1="38.1" x2="408.94" y2="35.56" width="0.1524" layer="91"/>
<junction x="408.94" y="35.56"/>
</segment>
<segment>
<pinref part="C59" gate="G$1" pin="1"/>
<pinref part="V62" gate="1" pin="+18V"/>
<wire x1="497.84" y1="38.1" x2="497.84" y2="35.56" width="0.1524" layer="91"/>
<junction x="497.84" y="35.56"/>
</segment>
<segment>
<pinref part="C58" gate="G$1" pin="1"/>
<pinref part="V61" gate="1" pin="+18V"/>
<wire x1="497.84" y1="83.82" x2="497.84" y2="81.28" width="0.1524" layer="91"/>
<junction x="497.84" y="81.28"/>
</segment>
<segment>
<pinref part="C57" gate="G$1" pin="1"/>
<pinref part="V60" gate="1" pin="+18V"/>
<wire x1="497.84" y1="129.54" x2="497.84" y2="127" width="0.1524" layer="91"/>
<junction x="497.84" y="127"/>
</segment>
<segment>
<pinref part="C50" gate="G$1" pin="1"/>
<pinref part="V53" gate="1" pin="+18V"/>
<wire x1="408.94" y1="83.82" x2="408.94" y2="81.28" width="0.1524" layer="91"/>
<junction x="408.94" y="81.28"/>
</segment>
<segment>
<pinref part="C49" gate="G$1" pin="1"/>
<pinref part="V52" gate="1" pin="+18V"/>
<wire x1="408.94" y1="129.54" x2="408.94" y2="127" width="0.1524" layer="91"/>
<junction x="408.94" y="127"/>
</segment>
<segment>
<pinref part="V20" gate="1" pin="+18V"/>
<wire x1="381" y1="139.7" x2="381" y2="144.78" width="0.1524" layer="91"/>
<label x="383.54" y="139.7" size="1.778" layer="95"/>
<pinref part="U17" gate="A" pin="VDD"/>
<wire x1="375.92" y1="139.7" x2="378.46" y2="139.7" width="0.1524" layer="91"/>
<pinref part="C17" gate="G$1" pin="1"/>
<wire x1="378.46" y1="139.7" x2="381" y2="139.7" width="0.1524" layer="91"/>
<wire x1="378.46" y1="129.54" x2="378.46" y2="139.7" width="0.1524" layer="91"/>
<junction x="378.46" y="139.7"/>
</segment>
<segment>
<wire x1="381" y1="93.98" x2="381" y2="99.06" width="0.1524" layer="91"/>
<pinref part="V21" gate="1" pin="+18V"/>
<label x="383.54" y="93.98" size="1.778" layer="95"/>
<pinref part="U18" gate="A" pin="VDD"/>
<wire x1="375.92" y1="93.98" x2="378.46" y2="93.98" width="0.1524" layer="91"/>
<pinref part="C18" gate="G$1" pin="1"/>
<wire x1="378.46" y1="93.98" x2="381" y2="93.98" width="0.1524" layer="91"/>
<wire x1="378.46" y1="83.82" x2="378.46" y2="93.98" width="0.1524" layer="91"/>
<junction x="378.46" y="93.98"/>
</segment>
<segment>
<wire x1="469.9" y1="93.98" x2="469.9" y2="99.06" width="0.1524" layer="91"/>
<pinref part="V29" gate="1" pin="+18V"/>
<label x="472.44" y="93.98" size="1.778" layer="95"/>
<pinref part="U26" gate="A" pin="VDD"/>
<wire x1="464.82" y1="93.98" x2="467.36" y2="93.98" width="0.1524" layer="91"/>
<pinref part="C26" gate="G$1" pin="1"/>
<wire x1="467.36" y1="93.98" x2="469.9" y2="93.98" width="0.1524" layer="91"/>
<wire x1="467.36" y1="83.82" x2="467.36" y2="93.98" width="0.1524" layer="91"/>
<junction x="467.36" y="93.98"/>
</segment>
<segment>
<pinref part="V28" gate="1" pin="+18V"/>
<wire x1="469.9" y1="139.7" x2="469.9" y2="144.78" width="0.1524" layer="91"/>
<label x="472.44" y="139.7" size="1.778" layer="95"/>
<pinref part="U25" gate="A" pin="VDD"/>
<wire x1="464.82" y1="139.7" x2="467.36" y2="139.7" width="0.1524" layer="91"/>
<pinref part="C25" gate="G$1" pin="1"/>
<wire x1="467.36" y1="139.7" x2="469.9" y2="139.7" width="0.1524" layer="91"/>
<wire x1="467.36" y1="129.54" x2="467.36" y2="139.7" width="0.1524" layer="91"/>
<junction x="467.36" y="139.7"/>
</segment>
</net>
<net name="PD40" class="4">
<segment>
<pinref part="U17" gate="A" pin="!OUT_A"/>
<wire x1="375.92" y1="134.62" x2="391.16" y2="134.62" width="0.1524" layer="91"/>
<wire x1="391.16" y1="134.62" x2="391.16" y2="139.7" width="0.1524" layer="91"/>
<pinref part="U$34" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="139.7" x2="396.24" y2="139.7" width="0.1524" layer="91"/>
<label x="381" y="134.62" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD36" class="4">
<segment>
<pinref part="U15" gate="A" pin="!OUT_A"/>
<wire x1="287.02" y1="-139.7" x2="302.26" y2="-139.7" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-139.7" x2="302.26" y2="-134.62" width="0.1524" layer="91"/>
<pinref part="U$30" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-134.62" x2="307.34" y2="-134.62" width="0.1524" layer="91"/>
<label x="292.1" y="-139.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD26" class="4">
<segment>
<pinref part="U11" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="40.64" x2="302.26" y2="40.64" width="0.1524" layer="91"/>
<wire x1="302.26" y1="40.64" x2="302.26" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$21" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="33.02" x2="307.34" y2="33.02" width="0.1524" layer="91"/>
<label x="292.1" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD50" class="4">
<segment>
<pinref part="U23" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="-142.24" x2="391.16" y2="-142.24" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-142.24" x2="391.16" y2="-149.86" width="0.1524" layer="91"/>
<pinref part="U$45" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-149.86" x2="396.24" y2="-149.86" width="0.1524" layer="91"/>
<label x="381" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD46" class="4">
<segment>
<pinref part="U21" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="-50.8" x2="391.16" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="391.16" y1="-50.8" x2="391.16" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="U$41" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="-58.42" x2="396.24" y2="-58.42" width="0.1524" layer="91"/>
<label x="381" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD42" class="4">
<segment>
<pinref part="U19" gate="A" pin="!OUT_B"/>
<wire x1="375.92" y1="40.64" x2="391.16" y2="40.64" width="0.1524" layer="91"/>
<wire x1="391.16" y1="40.64" x2="391.16" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$37" gate="G$1" pin="P0"/>
<wire x1="391.16" y1="33.02" x2="396.24" y2="33.02" width="0.1524" layer="91"/>
<label x="381" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD30" class="4">
<segment>
<pinref part="U13" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="-50.8" x2="302.26" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="302.26" y1="-50.8" x2="302.26" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="U$25" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="-58.42" x2="307.34" y2="-58.42" width="0.1524" layer="91"/>
<label x="292.1" y="-50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD22" class="4">
<segment>
<pinref part="U9" gate="A" pin="!OUT_B"/>
<wire x1="287.02" y1="132.08" x2="302.26" y2="132.08" width="0.1524" layer="91"/>
<wire x1="302.26" y1="132.08" x2="302.26" y2="124.46" width="0.1524" layer="91"/>
<pinref part="U$17" gate="G$1" pin="P0"/>
<wire x1="302.26" y1="124.46" x2="307.34" y2="124.46" width="0.1524" layer="91"/>
<label x="292.1" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD6" class="4">
<segment>
<pinref part="U1" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="132.08" x2="210.82" y2="132.08" width="0.1524" layer="91"/>
<wire x1="210.82" y1="132.08" x2="210.82" y2="124.46" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="124.46" x2="215.9" y2="124.46" width="0.1524" layer="91"/>
<label x="200.66" y="132.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD8" class="4">
<segment>
<pinref part="U2" gate="A" pin="!OUT_B"/>
<wire x1="195.58" y1="86.36" x2="210.82" y2="86.36" width="0.1524" layer="91"/>
<wire x1="210.82" y1="86.36" x2="210.82" y2="78.74" width="0.1524" layer="91"/>
<pinref part="U$5" gate="G$1" pin="P0"/>
<wire x1="210.82" y1="78.74" x2="215.9" y2="78.74" width="0.1524" layer="91"/>
<label x="200.66" y="86.36" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD64" class="4">
<segment>
<pinref part="U30" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="-96.52" x2="480.06" y2="-96.52" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-96.52" x2="480.06" y2="-104.14" width="0.1524" layer="91"/>
<pinref part="U$59" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-104.14" x2="485.14" y2="-104.14" width="0.1524" layer="91"/>
<label x="469.9" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD66" class="4">
<segment>
<pinref part="U31" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="-142.24" x2="480.06" y2="-142.24" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-142.24" x2="480.06" y2="-149.86" width="0.1524" layer="91"/>
<pinref part="U$61" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-149.86" x2="485.14" y2="-149.86" width="0.1524" layer="91"/>
<label x="469.9" y="-142.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="PD68" class="4">
<segment>
<pinref part="U32" gate="A" pin="!OUT_B"/>
<wire x1="464.82" y1="-187.96" x2="480.06" y2="-187.96" width="0.1524" layer="91"/>
<wire x1="480.06" y1="-187.96" x2="480.06" y2="-195.58" width="0.1524" layer="91"/>
<pinref part="U$63" gate="G$1" pin="P0"/>
<wire x1="480.06" y1="-195.58" x2="485.14" y2="-195.58" width="0.1524" layer="91"/>
<label x="469.9" y="-187.96" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
<note version="8.4" severity="warning">
Since Version 8.4, EAGLE supports properties for SPICE simulation. 
Probes in schematics and SPICE mapping objects found in parts and library devices
will not be understood with this version. Update EAGLE to the latest version
for full support of SPICE simulation. 
</note>
</compatibility>
</eagle>
